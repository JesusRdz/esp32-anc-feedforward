# ESP32 ANC Feedforward

Sistema de **Cancelación Activa de Ruido** (ANC) tipo feedforward para ESP32, usando un micrófono digital INMP441 y un amplificador/DAC I2S MAX98357A.

## Diagrama de señal

```
Ruido → [INMP441] →(I2S RX)→ [ESP32 DSP] →(I2S TX)→ [MAX98357A] → Altavoz
                                   │
                      Inversión de fase
                      Filtro pasa-bajas
                      Bloqueador DC
                      Puerta de ruido
                      Ganancia ajustable
```

## Hardware necesario

| Componente | Descripción |
|---|---|
| ESP32 DevKit V1 | Microcontrolador (u otro ESP32 con 2 periféricos I2S) |
| INMP441 | Micrófono MEMS digital I2S |
| MAX98357A | Módulo DAC + amplificador I2S clase D (3W) |
| Altavoz | 4–8 Ω |

## Conexiones

### INMP441 → ESP32 (I2S_NUM_1)

| INMP441 | ESP32 | Nota |
|---|---|---|
| VDD | 3.3V | |
| GND | GND | |
| SCK | GPIO 32 | Serial Clock (BCLK) |
| WS | GPIO 14 | Word Select (LRCK) |
| SD | GPIO 33 | Serial Data Out |
| L/R | GND | Canal izquierdo (si lo conectas a 3.3V, cambia `I2S_CHANNEL_FMT_ONLY_LEFT` a `ONLY_RIGHT` en el código) |

### MAX98357A → ESP32 (I2S_NUM_0)

| MAX98357A | ESP32 | Nota |
|---|---|---|
| VIN | 5V (o 3.3V) | Alimentación del módulo |
| GND | GND | Referencia común con ESP32 e INMP441 |
| BCLK | GPIO 25 | Bit Clock |
| LRC | GPIO 26 | Left/Right Clock (Word Select) |
| DIN | GPIO 27 | Data In (señal de audio I2S) |
| SD | sin conectar o VIN | Shutdown: flotante = 3W mono izq. Conectar a VIN = siempre encendido. |
| GAIN | sin conectar | Flotante = 9 dB. Ver tabla abajo para otros valores. |

#### Configuración de ganancia del MAX98357A (pin GAIN)

| Conexión del pin GAIN | Ganancia |
|---|---|
| Flotante (sin conectar) | 9 dB |
| GND | 12 dB |
| VIN | 15 dB |

> **Importante:** Conecta todos los GND entre sí: ESP32, INMP441 y MAX98357A. Sin referencia común, habrá ruido.

## Arquitectura del software

```
I2S_NUM_1 (RX)              I2S_NUM_0 (TX)
    │                           ▲
    │  INMP441 → DMA            │  DMA → MAX98357A
    ▼                           │
┌──────────────────────────────────┐
│          Cadena DSP              │
│  1. Extraer 24 bits (INMP441)   │
│  2. Bloqueador DC               │
│  3. Filtro pasa-bajas IIR       │
│  4. Puerta de ruido (noise gate)│
│  5. Inversión de fase (×-1)     │
│  6. Ganancia ajustable          │
│  7. Escalar a 16 bits con signo │
└──────────────────────────────────┘
```

### ¿Por qué dos periféricos I2S?

El ESP32 tiene dos periféricos I2S (NUM_0 y NUM_1). Usar uno para entrada (INMP441) y otro para salida (MAX98357A) permite transferencias DMA completamente independientes, minimizando latencia y jitter.

### Ventaja del MAX98357A sobre el DAC interno

| Característica | DAC interno ESP32 | MAX98357A |
|---|---|---|
| Resolución | 8 bits (~48 dB) | 16 bits (~96 dB) |
| Interfaz | Analógica | I2S digital (sin ruido de conversión) |
| Amplificación | Necesita amplificador externo | Integrada (3W clase D) |
| Jitter | Depende del método de escritura | DMA I2S nativo |

## Instalación

1. Abre el archivo `esp32-anc-feedforward.ino` en el **Arduino IDE** (≥ 1.8) o **PlatformIO**.
2. Instala el soporte para ESP32 si no lo tienes:
   - Arduino IDE: *Archivo → Preferencias → URLs adicionales* → `https://dl.espressif.com/dl/package_esp32_index.json`
   - Gestor de tarjetas: busca "esp32" e instala.
3. Selecciona tu placa: **ESP32 Dev Module**.
4. Conecta, compila y carga (`Ctrl+U`).

## Ajuste en tiempo real (Serial)

Abre el Monitor Serial a **115200 baud**. Comandos disponibles:

| Comando | Descripción | Ejemplo |
|---|---|---|
| `gain <valor>` | Ganancia de salida (0.0–2.0) | `gain 1.2` |
| `lp <valor>` | Coeficiente pasa-bajas (0.01–1.0). Menor = más filtrado. | `lp 0.10` |
| `gate <valor>` | Umbral de puerta de ruido. Mayor = más silencio. | `gate 1000` |
| `mute` | Silencia la salida (gain = 0) | `mute` |
| `diag` | Muestra niveles del micrófono por 5 segundos | `diag` |
| `status` | Muestra parámetros actuales | `status` |

### Guía de ajuste

1. **Empieza con ganancia baja** (`gain 0.3`) y sube gradualmente.
2. **Si hay retroalimentación** (pitido agudo), baja la ganancia inmediatamente.
3. **Para ruido grave** (ventilador, motor), usa `lp 0.10`–`0.20` para filtrar frecuencias altas.
4. **Para ruido de banda ancha**, sube `lp` a `0.40`–`0.60`.
5. **Usa `diag`** para verificar que el micrófono está captando señal (max > 0).
6. La frecuencia de corte aproximada es: `fc ≈ (alpha × 16000) / (2π)` Hz.

| alpha | fc aprox. |
|---|---|
| 0.10 | ~255 Hz |
| 0.15 | ~382 Hz |
| 0.25 | ~637 Hz |
| 0.40 | ~1018 Hz |

## Latencia del sistema

| Etapa | Latencia estimada |
|---|---|
| DMA entrada (64 muestras @ 16 kHz) | ~4 ms |
| Procesamiento DSP | < 0.1 ms |
| DMA salida (64 muestras @ 16 kHz) | ~4 ms |
| **Total electrónico** | **~8 ms** |
| Propagación acústica (depende de distancia) | Variable |

Para reducir latencia, puedes bajar `BLOCK_SIZE` a 32 (aumenta carga de CPU) o subir `SAMPLE_RATE` a 32000.

## Limitaciones

1. **Sin filtro adaptativo**: Este sistema usa inversión de fase directa (feedforward sin error mic). No se adapta al entorno acústico. Para un ANC robusto se necesita un segundo micrófono (error) y un algoritmo FxLMS.
2. **Retardo no compensado**: La cancelación es efectiva solo si la distancia entre micrófono y altavoz es pequeña (< 5 cm idealmente) para que la latencia del sistema sea menor que medio periodo de la frecuencia a cancelar.
3. **Frecuencias cancelables**: Con 8 ms de latencia, la cancelación es más efectiva por debajo de ~200–300 Hz en la práctica.

## Mejoras futuras

- [ ] Agregar segundo micrófono (error) para filtro adaptativo FxLMS
- [ ] Implementar filtro FIR configurable para modelar el camino acústico secundario
- [ ] Añadir análisis de espectro vía Serial para diagnóstico
- [ ] Soporte para sample rate más alto (32/44.1 kHz)

## Licencia

MIT
