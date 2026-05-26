// ============================================================================
// ESP32 ANC Feedforward — INMP441 + TDA2030A
// ============================================================================
// Sistema de Cancelación Activa de Ruido (feedforward) con un solo micrófono.
// Entrada: INMP441 (I2S digital) → DSP (inversión de fase + filtros) →
// DAC interno (GPIO 25) → TDA2030A → Altavoz
//
// Hardware:
//   - ESP32 DevKit V1 (o compatible con 2 periféricos I2S)
//   - INMP441 micrófono MEMS I2S
//   - TDA2030A módulo amplificador
//   - Altavoz 4-8 Ω
//
// Periféricos I2S:
//   I2S_NUM_0 → DAC built-in (solo I2S0 soporta DAC interno en ESP32)
//   I2S_NUM_1 → Entrada digital desde INMP441
// ============================================================================

#include <driver/i2s.h>

// ============================================================================
// CONFIGURACIÓN DE PINES
// ============================================================================
// INMP441 → I2S_NUM_1
static const int MIC_BCLK = 32;  // Serial Clock (SCK/BCLK)
static const int MIC_WS   = 14;  // Word Select  (LRCK) — GPIO 14 libera GPIO 25 para DAC
static const int MIC_SD   = 33;  // Serial Data  (DOUT)

// DAC1 (GPIO 25) → TDA2030A
// Cableado interno del ESP32; no requiere asignación de pin.

// ============================================================================
// PARÁMETROS DE AUDIO
// ============================================================================
static const uint32_t SAMPLE_RATE   = 16000;  // Hz — buen balance latencia/calidad
static const int      BLOCK_SIZE    = 64;     // Muestras por bloque DMA
static const int      DMA_BUF_COUNT = 4;      // Buffers DMA (doble del mínimo)

// ============================================================================
// PARÁMETROS ANC — ajustables en tiempo real vía Serial
// ============================================================================
static float g_gain     = 0.85f;   // Ganancia de salida [0.0 – 2.0]
static float g_lp_alpha = 0.25f;   // Filtro pasa-bajas: menor → más filtrado
static const float DC_BLOCK_R = 0.995f;  // Constante del bloqueador DC

// ============================================================================
// BUFFERS
// ============================================================================
static int32_t  rx_buf[BLOCK_SIZE];   // Entrada cruda (32 bits por muestra)
static uint16_t tx_buf[BLOCK_SIZE];   // Salida empaquetada para DAC I2S (16 bits)

// ============================================================================
// ESTADO DE FILTROS
// ============================================================================
static float dc_x1 = 0.0f;   // Entrada anterior del bloqueador DC
static float dc_y1 = 0.0f;   // Salida anterior del bloqueador DC
static float lp_y1 = 0.0f;   // Salida anterior del pasa-bajas

// ============================================================================
// INICIALIZACIÓN DE PERIFÉRICOS I2S
// ============================================================================

static void init_mic() {
  i2s_config_t cfg = {};
  cfg.mode             = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
  cfg.sample_rate      = SAMPLE_RATE;
  cfg.bits_per_sample  = I2S_BITS_PER_SAMPLE_32BIT;
  cfg.channel_format   = I2S_CHANNEL_FMT_ONLY_LEFT;  // INMP441 con L/R → GND
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count    = DMA_BUF_COUNT;
  cfg.dma_buf_len      = BLOCK_SIZE;
  cfg.use_apll         = true;   // Reloj APLL para menor jitter
  cfg.tx_desc_auto_clear = false;

  i2s_pin_config_t pins = {};
  pins.bck_io_num   = MIC_BCLK;
  pins.ws_io_num    = MIC_WS;
  pins.data_out_num = I2S_PIN_NO_CHANGE;
  pins.data_in_num  = MIC_SD;

  ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_1, &cfg, 0, NULL));
  ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM_1, &pins));
}

static void init_dac() {
  i2s_config_t cfg = {};
  cfg.mode             = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN);
  cfg.sample_rate      = SAMPLE_RATE;
  cfg.bits_per_sample  = I2S_BITS_PER_SAMPLE_16BIT;  // DAC 8-bit usa word de 16 bits
  cfg.channel_format   = I2S_CHANNEL_FMT_ONLY_RIGHT;  // DAC1 = canal derecho
  cfg.communication_format = I2S_COMM_FORMAT_STAND_MSB;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count    = DMA_BUF_COUNT;
  cfg.dma_buf_len      = BLOCK_SIZE;
  cfg.use_apll         = true;
  cfg.tx_desc_auto_clear = true;

  ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL));
  ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM_0, NULL));      // DAC interno, sin pines externos
  i2s_set_dac_mode(I2S_DAC_CHANNEL_RIGHT_EN);          // Habilita DAC1 → GPIO 25
}

// ============================================================================
// FILTROS DSP (inline para máxima velocidad)
// ============================================================================

// Bloqueador DC — elimina el offset de continua del INMP441
// H(z) = (1 - z⁻¹) / (1 - R·z⁻¹)
static inline float dc_block(float x) {
  float y = x - dc_x1 + DC_BLOCK_R * dc_y1;
  dc_x1 = x;
  dc_y1 = y;
  return y;
}

// Pasa-bajas IIR de primer orden — atenúa frecuencias altas que el sistema
// no puede cancelar efectivamente (limitación del DAC 8-bit y latencia)
// fc ≈ (alpha × fs) / (2π) → alpha=0.25, fs=16 kHz → fc ≈ 637 Hz
static inline float low_pass(float x) {
  lp_y1 += g_lp_alpha * (x - lp_y1);
  return lp_y1;
}

// Saturación segura a rango [0, 255]
static inline uint8_t clamp8(int32_t v) {
  if (v > 255) return 255;
  if (v < 0)   return 0;
  return (uint8_t)v;
}

// ============================================================================
// INTERFAZ SERIAL — ajuste en tiempo real
// ============================================================================

static void process_serial() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd.startsWith("gain ")) {
    g_gain = cmd.substring(5).toFloat();
    Serial.printf(">> Ganancia = %.2f\n", g_gain);
  } else if (cmd.startsWith("lp ")) {
    g_lp_alpha = cmd.substring(3).toFloat();
    Serial.printf(">> LP alpha = %.3f (fc ~ %.0f Hz)\n",
                  g_lp_alpha, g_lp_alpha * SAMPLE_RATE / 6.2832f);
  } else if (cmd == "status") {
    Serial.printf("Gain=%.2f  LP_alpha=%.3f  SR=%u  Block=%d\n",
                  g_gain, g_lp_alpha, SAMPLE_RATE, BLOCK_SIZE);
  } else {
    Serial.println("Comandos: gain <0.0-2.0> | lp <0.01-1.0> | status");
  }
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);

  init_dac();   // I2S_NUM_0 — salida DAC
  init_mic();   // I2S_NUM_1 — entrada micrófono

  Serial.println("=== ESP32 ANC Feedforward ===");
  Serial.printf("INMP441 → DSP → DAC1 (GPIO 25) → TDA2030A\n");
  Serial.printf("Sample Rate: %u Hz | Block: %d | Gain: %.2f\n",
                SAMPLE_RATE, BLOCK_SIZE, g_gain);
  Serial.println("Comandos Serial: gain <val> | lp <val> | status");
  Serial.println("=============================");
}

// ============================================================================
// LOOP PRINCIPAL
// ============================================================================
void loop() {
  size_t bytes_read    = 0;
  size_t bytes_written = 0;

  // --- LECTURA: DMA llena rx_buf con datos del INMP441 ---
  i2s_read(I2S_NUM_1, rx_buf, sizeof(rx_buf), &bytes_read, portMAX_DELAY);

  const int n = bytes_read / sizeof(int32_t);

  // --- PROCESAMIENTO DSP + EMPAQUETADO DAC ---
  for (int i = 0; i < n; i++) {
    // 1. Extraer 24 bits significativos del INMP441 (justificados a la izquierda)
    float sample = (float)(rx_buf[i] >> 8);

    // 2. Eliminar offset DC
    sample = dc_block(sample);

    // 3. Filtro pasa-bajas (focalizar energía cancelable)
    sample = low_pass(sample);

    // 4. Inversión de fase + ganancia
    sample = -sample * g_gain;

    // 5. Escalar de ~24 bits a 8 bits y centrar en 128 (punto medio DAC)
    int32_t dac_val = (int32_t)(sample / 65536.0f) + 128;

    // 6. Saturar y empaquetar para I2S DAC (valor en bits [15:8])
    tx_buf[i] = (uint16_t)(clamp8(dac_val) << 8);
  }

  // --- ESCRITURA: DMA envía tx_buf al DAC (sin jitter) ---
  i2s_write(I2S_NUM_0, tx_buf, n * sizeof(uint16_t), &bytes_written, portMAX_DELAY);

  // --- Comandos serial (no bloqueante) ---
  process_serial();
}
