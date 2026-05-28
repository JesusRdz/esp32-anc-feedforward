// ============================================================================
// ESP32 ANC Feedforward — INMP441 + MAX98357A
// ============================================================================
// Sistema de Cancelación Activa de Ruido (feedforward) con un solo micrófono.
// Entrada: INMP441 (I2S digital) → DSP (inversión de fase + filtros) →
// MAX98357A (I2S DAC+Amp) → Altavoz
//
// Hardware:
//   - ESP32 DevKit V1 (o compatible con 2 periféricos I2S)
//   - INMP441 micrófono MEMS I2S
//   - MAX98357A módulo DAC + amplificador I2S (3W clase D)
//   - Altavoz 4-8 Ω
//
// Periféricos I2S:
//   I2S_NUM_0 → TX I2S estándar → MAX98357A
//   I2S_NUM_1 → RX I2S estándar ← INMP441
// ============================================================================

#include <driver/i2s.h>

// ============================================================================
// CONFIGURACIÓN DE PINES
// ============================================================================
// INMP441 → I2S_NUM_1
static const int MIC_BCLK = 32;  // Serial Clock (SCK/BCLK)
static const int MIC_WS   = 14;  // Word Select  (LRCK)
static const int MIC_SD   = 33;  // Serial Data  (DOUT)

// MAX98357A → I2S_NUM_0
static const int SPK_BCLK = 25;  // Bit Clock
static const int SPK_LRC  = 26;  // Left/Right Clock (Word Select)
static const int SPK_DIN  = 27;  // Data In

// ============================================================================
// PARÁMETROS DE AUDIO
// ============================================================================
static const uint32_t SAMPLE_RATE   = 16000;  // Hz
static const int      BLOCK_SIZE    = 64;     // Muestras por bloque DMA
static const int      DMA_BUF_COUNT = 4;      // Buffers DMA

// ============================================================================
// PARÁMETROS ANC — ajustables en tiempo real vía Serial
// ============================================================================
static float g_gain       = 0.30f;    // Ganancia de salida [0.0 – 2.0]
static float g_lp_alpha   = 0.25f;    // Filtro pasa-bajas: menor → más filtrado
static float g_noise_gate = 500.0f;   // Umbral de puerta de ruido
static const float DC_BLOCK_R = 0.995f;

// ============================================================================
// BUFFERS
// ============================================================================
static int32_t rx_buf[BLOCK_SIZE];    // Entrada cruda del INMP441 (32 bits)
static int16_t tx_buf[BLOCK_SIZE];    // Salida I2S para MAX98357A (16 bits con signo)

// ============================================================================
// ESTADO DE FILTROS Y SISTEMA
// ============================================================================
static float dc_x1 = 0.0f;
static float dc_y1 = 0.0f;
static float lp_y1 = 0.0f;

static uint32_t startup_counter = 0;
static const uint32_t MUTE_BLOCKS = (SAMPLE_RATE / BLOCK_SIZE);  // ~1s de silencio

static uint32_t diag_counter = 0;

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
  cfg.use_apll         = true;
  cfg.tx_desc_auto_clear = false;

  i2s_pin_config_t pins = {};
  pins.bck_io_num   = MIC_BCLK;
  pins.ws_io_num    = MIC_WS;
  pins.data_out_num = I2S_PIN_NO_CHANGE;
  pins.data_in_num  = MIC_SD;

  ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_1, &cfg, 0, NULL));
  ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM_1, &pins));
}

static void init_spk() {
  i2s_config_t cfg = {};
  cfg.mode             = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  cfg.sample_rate      = SAMPLE_RATE;
  cfg.bits_per_sample  = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format   = I2S_CHANNEL_FMT_ONLY_LEFT;   // MAX98357A mono (canal izquierdo)
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count    = DMA_BUF_COUNT;
  cfg.dma_buf_len      = BLOCK_SIZE;
  cfg.use_apll         = true;
  cfg.tx_desc_auto_clear = true;

  i2s_pin_config_t pins = {};
  pins.bck_io_num   = SPK_BCLK;
  pins.ws_io_num    = SPK_LRC;
  pins.data_out_num = SPK_DIN;
  pins.data_in_num  = I2S_PIN_NO_CHANGE;

  ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL));
  ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM_0, &pins));
}

// ============================================================================
// FILTROS DSP (inline para máxima velocidad)
// ============================================================================

// Bloqueador DC — elimina offset de continua del INMP441
// H(z) = (1 - z⁻¹) / (1 - R·z⁻¹)
static inline float dc_block(float x) {
  float y = x - dc_x1 + DC_BLOCK_R * dc_y1;
  dc_x1 = x;
  dc_y1 = y;
  return y;
}

// Pasa-bajas IIR de primer orden
// fc ≈ (alpha × fs) / (2π) → alpha=0.25, fs=16 kHz → fc ≈ 637 Hz
static inline float low_pass(float x) {
  lp_y1 += g_lp_alpha * (x - lp_y1);
  return lp_y1;
}

// Saturación a rango int16 [-32768, 32767]
static inline int16_t clamp16(int32_t v) {
  if (v > 32767)  return 32767;
  if (v < -32768) return -32768;
  return (int16_t)v;
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
  } else if (cmd.startsWith("gate ")) {
    g_noise_gate = cmd.substring(5).toFloat();
    Serial.printf(">> Noise gate = %.0f\n", g_noise_gate);
  } else if (cmd == "mute") {
    g_gain = 0.0f;
    Serial.println(">> MUTE (gain = 0)");
  } else if (cmd == "diag") {
    Serial.println(">> Diagnostico activado (5 segundos)");
    diag_counter = 5 * (SAMPLE_RATE / BLOCK_SIZE);
  } else if (cmd == "status") {
    Serial.printf("Gain=%.2f  LP=%.3f  Gate=%.0f  SR=%u  Block=%d\n",
                  g_gain, g_lp_alpha, g_noise_gate, SAMPLE_RATE, BLOCK_SIZE);
  } else {
    Serial.println("Comandos: gain <val> | lp <val> | gate <val> | mute | diag | status");
  }
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);

  init_spk();   // I2S_NUM_0 — salida MAX98357A
  init_mic();   // I2S_NUM_1 — entrada INMP441

  Serial.println("=== ESP32 ANC Feedforward ===");
  Serial.println("INMP441 -> DSP -> MAX98357A (I2S 16-bit) -> Altavoz");
  Serial.printf("Sample Rate: %u Hz | Block: %d | Gain: %.2f\n",
                SAMPLE_RATE, BLOCK_SIZE, g_gain);
  Serial.println("Comandos: gain | lp | gate | mute | diag | status");
  Serial.printf("Silencio inicial: ~1 segundo (estabilizacion de filtros)\n");
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

  // --- Periodo de silencio al arranque (estabiliza filtros DC y LP) ---
  if (startup_counter < MUTE_BLOCKS) {
    startup_counter++;
    for (int i = 0; i < n; i++) {
      float sample = (float)(rx_buf[i] >> 8);
      dc_block(sample);
      low_pass(sample);
      tx_buf[i] = 0;  // Silencio = 0 en PCM con signo
    }
    i2s_write(I2S_NUM_0, tx_buf, n * sizeof(int16_t), &bytes_written, portMAX_DELAY);
    return;
  }

  // --- Variables para diagnóstico ---
  float block_max = 0.0f;
  float block_sum = 0.0f;

  // --- PROCESAMIENTO DSP ---
  for (int i = 0; i < n; i++) {
    // 1. Extraer 24 bits significativos del INMP441 (justificados a la izquierda)
    float sample = (float)(rx_buf[i] >> 8);

    // 2. Eliminar offset DC
    sample = dc_block(sample);

    // 3. Filtro pasa-bajas
    sample = low_pass(sample);

    // Diagnóstico
    float abs_sample = (sample < 0) ? -sample : sample;
    if (abs_sample > block_max) block_max = abs_sample;
    block_sum += abs_sample;

    // 4. Puerta de ruido
    if (abs_sample < g_noise_gate) {
      tx_buf[i] = 0;
      continue;
    }

    // 5. Inversión de fase + ganancia
    sample = -sample * g_gain;

    // 6. Escalar de ~24 bits a 16 bits con signo
    int32_t out_val = (int32_t)(sample / 256.0f);

    // 7. Saturar a rango int16
    tx_buf[i] = clamp16(out_val);
  }

  // --- ESCRITURA: DMA envía tx_buf al MAX98357A ---
  i2s_write(I2S_NUM_0, tx_buf, n * sizeof(int16_t), &bytes_written, portMAX_DELAY);

  // --- Diagnóstico periódico ---
  if (diag_counter > 0) {
    diag_counter--;
    float block_avg = (n > 0) ? block_sum / n : 0;
    Serial.printf("[DIAG] max=%.0f avg=%.0f gate=%.0f %s\n",
                  block_max, block_avg, g_noise_gate,
                  (block_max > g_noise_gate) ? "ACTIVO" : "SILENCIO");
  }

  // --- Comandos serial ---
  process_serial();
}
