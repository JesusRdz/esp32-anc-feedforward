# ESP32 ANC Feedforward

Sistema de **Cancelación Activa de Ruido** (ANC) tipo feedforward para ESP32, usando un micrófono digital INMP441 y un amplificador TDA2030A.

## Diagrama de señal

```
Ruido → [INMP441] →(I2S)→ [ESP32 DSP] →(DAC GPIO 25)→ [TDA2030A] → Altavoz
              │                  │
              │    Inversión de fase
              │    Filtro pasa-bajas
              │    Bloqueador DC
              │    Ganancia ajustable
```

## Hardware necesario

| Componente | Descripción |
|---|---|
| ESP32 DevKit V1 | Microcontrolador (u otro ESP32 con 2 periféricos I2S) |
| INMP441 | Micrófono MEMS digital I2S |
| TDA2030A | Módulo amplificador de audio |
| Altavoz | 4–8 Ω |
| Fuente | 5V para ESP32, 6–18V para TDA2030A (según módulo) |

## Conexiones

### INMP441 → ESP32

| INMP441 | ESP32 | Nota |
|---|---|---|
| VDD | 3.3V | |
| GND | GND | |
| SCK | GPIO 32 | Serial Clock (BCLK) |
| WS | GPIO 14 | Word Select (LRCK) — no usar GPIO 25, está reservado para DAC |
| SD | GPIO 33 | Serial Data Out |
| L/R | GND | Canal izquierdo (si lo conectas a 3.3V, cambia `I2S_CHANNEL_FMT_ONLY_LEFT` a `ONLY_RIGHT` en el código) |

### ESP32 → TDA2030A

| ESP32 | TDA2030A | Nota |
|---|---|---|
| GPIO 25 | Entrada de audio (IN) | Salida DAC1 del ESP32 |
| GND | GND | Referencia común obligatoria |

> **Importante:** Conecta las tierras (GND) del ESP32 y el TDA2030A entre sí. Sin referencia común, habrá ruido excesivo.

## Arquitectura del software

```
I2S_NUM_1 (RX)          I2S_NUM_0 (TX + DAC_BUILT_IN)
    │                           ▲
    │  INMP441 → DMA            │  DMA → DAC1 (GPIO 25)
    ▼                           │
┌─────────────────────────────────┐
│         Cadena DSP              │
│  1. Extraer 24 bits            │
│  2. Bloqueador DC              │
│  3. Filtro pasa-bajas IIR      │
│  4. Inversión de fase (×-1)    │
│  5. Ganancia ajustable         │
│  6. Escalar a 8 bits + offset  │
│  7. Empaquetar para DAC I2S    │
└─────────────────────────────────┘
```

### ¿Por qué dos periféricos I2S?

El ESP32 tiene dos periféricos I2S (NUM_0 y NUM_1), pero **solo I2S_NUM_0 soporta el DAC interno**. Usar ambos periféricos por separado evita conflictos entre RX y TX en el mismo bus y permite transferencias DMA independientes, reduciendo latencia y jitter.

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
| `status` | Muestra parámetros actuales | `status` |

### Guía de ajuste

1. **Empieza con ganancia baja** (`gain 0.3`) y sube gradualmente.
2. **Si hay retroalimentación** (pitido agudo), baja la ganancia inmediatamente.
3. **Para ruido grave** (ventilador, motor), usa `lp 0.10`–`0.20` para filtrar frecuencias altas.
4. **Para ruido de banda ancha**, sube `lp` a `0.40`–`0.60`.
5. La frecuencia de corte aproximada es: `fc ≈ (alpha × 16000) / (2π)` Hz.

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

1. **DAC de 8 bits**: Rango dinámico de ~48 dB. Para mejor rendimiento, considera un DAC externo I2S (PCM5102, MAX98357A) conectado al TDA2030A.
2. **Sin filtro adaptativo**: Este sistema usa inversión de fase directa (feedforward sin error mic). No se adapta al entorno acústico. Para un ANC robusto se necesita un segundo micrófono (error) y un algoritmo FxLMS.
3. **Retardo no compensado**: La cancelación es efectiva solo si la distancia entre micrófono y altavoz es pequeña (< 5 cm idealmente) para que la latencia del sistema sea menor que medio periodo de la frecuencia a cancelar.
4. **Frecuencias cancelables**: Con 8 ms de latencia, la cancelación es más efectiva por debajo de ~60 Hz teóricamente. En la práctica, la atenuación parcial puede notarse hasta ~200–300 Hz.

## Mejoras futuras

- [ ] Agregar segundo micrófono (error) para filtro adaptativo FxLMS
- [ ] Usar DAC externo I2S de 16 bits para mayor rango dinámico
- [ ] Implementar filtro FIR configurable para modelar el camino acústico secundario
- [ ] Añadir análisis de espectro vía Serial para diagnóstico

## Licencia

MIT
