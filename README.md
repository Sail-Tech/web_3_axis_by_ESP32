# ESP32 + ICM-20948 — Visualizador 3D de Orientação (9 eixos)

Visualização em tempo real da orientação de um sensor IMU ICM-20948 ligado a um ESP32 via SPI, com interface web 3D servida pelo próprio microcontrolador. Sem dependências externas, sem CDN, funciona em modo Access Point sem internet.

![Eixos](https://img.shields.io/badge/eixos-9_(accel+gyro+mag)-blue)
![Protocolo](https://img.shields.io/badge/protocolo-WebSocket_50Hz-green)
![SPI](https://img.shields.io/badge/SPI-Mode_3-orange)

---

## Funcionalidades

- **Visualização 3D** das setas X/Y/Z com projeção isométrica (todos os 3 eixos sempre visíveis)
- **9 eixos em tempo real**: acelerómetro (g), giroscópio (°/s), magnetómetro (µT)
- **Complementary filter** para fusão accel+gyro → quaternião de orientação
- **WebSocket a 50 Hz** para baixa latência entre sensor e browser
- **Modo AP** integrado — o ESP32 cria a sua própria rede WiFi, sem router necessário
- **Zero dependências web** — todo o HTML/CSS/JS é servido do ESP32, sem CDN
- **Modo simulação** no browser para testar a visualização sem sensor
- **Driver SPI próprio** em Mode 3, sem dependência da library SparkFun

---

## Hardware

### Componentes

| Componente | Descrição |
|---|---|
| ESP32 DevKit | Qualquer variante com SPI disponível (VSPI) |
| ICM-20948 | Módulo breakout (SparkFun, Adafruit, genérico) |
| Fios/jumpers | 5 ligações SPI + alimentação |

### Ligações SPI

| ESP32 | ICM-20948 | Função |
|---|---|---|
| GPIO 18 | SCK / SCLK | Clock |
| GPIO 19 | MISO / SDO | Data Out (sensor → ESP32) |
| GPIO 23 | MOSI / SDI / SDA | Data In (ESP32 → sensor) |
| GPIO 5 | CS / SS | Chip Select |
| 3.3V | VCC / 3V3 | Alimentação |
| GND | GND | Massa |

> **Nota:** Alguns módulos marcam o pino de dados como SDA (usado tanto em I2C como SPI). Confirma no datasheet do teu módulo específico.

### Esquema

```
ESP32                    ICM-20948
┌──────────┐            ┌──────────┐
│     3V3  ├────────────┤ VCC      │
│     GND  ├────────────┤ GND      │
│  GPIO 18 ├────────────┤ SCK      │
│  GPIO 23 ├────────────┤ MOSI/SDI │
│  GPIO 19 ├────────────┤ MISO/SDO │
│  GPIO  5 ├────────────┤ CS       │
└──────────┘            └──────────┘
```

---

## Software

### Requisitos

- [Arduino IDE](https://www.arduino.cc/en/software) 2.x ou PlatformIO
- Board package **ESP32 by Espressif** (Board Manager → procurar "esp32")
- Library **WebSockets** by Markus Sattler (Library Manager → procurar "WebSockets")

> **Não é necessária** a library SparkFun ICM-20948. O sketch usa um driver SPI próprio.

### Instalação

1. Abre o Arduino IDE
2. Instala o board package ESP32: `Tools → Board → Boards Manager → "esp32"`
3. Instala a library WebSockets: `Sketch → Include Library → Manage Libraries → "WebSockets"`
4. Abre o ficheiro `ICM20948_Mode3.ino`
5. Seleciona a board: `Tools → Board → ESP32 Dev Module`
6. Seleciona a porta COM
7. Faz Upload

### Configuração WiFi

Por defeito, o ESP32 cria um Access Point:

| Parâmetro | Valor |
|---|---|
| SSID | `ICM20948-3D` |
| Password | `12345678` |
| IP | `192.168.4.1` |

Para ligar a uma rede WiFi existente, edita as variáveis no topo do sketch:

```cpp
const char* WIFI_SSID = "a-tua-rede";
const char* WIFI_PASS = "a-tua-password";
```

---

## Utilização

1. Faz upload do sketch para o ESP32
2. Liga ao WiFi `ICM20948-3D` (password: `12345678`)
3. Abre `http://192.168.4.1` no browser
4. Roda o sensor lentamente — as setas 3D acompanham a orientação

### Interface Web

- **Painel esquerdo**: estado da ligação, quaternião, FPS, e barras dos 9 eixos raw
- **Canvas central**: visualização 3D com 3 setas (X vermelho, Y verde, Z azul)
- **Botão Simulação**: testa a visualização com rotação automática (sem dados do sensor)
- **Botão Reset**: volta à orientação inicial (quaternião identidade)
- **Botão Reconnect**: força nova ligação WebSocket

### Serial Monitor

Abre o Serial Monitor a **115200 baud** para diagnóstico:

```
[ICM] WHO_AM_I = 0xEA → ICM-20948 OK!
[ICM] Test: Acc=[123,-456,16234] Gyr=[12,-8,3]
[ICM] Sensor ready!
A[0.01 -0.03 1.00]g  G[1 -1 0]  M[25 -8 42]  Q[1.000 0.001 -0.015 0.000]
```

---

## Arquitetura

```
┌─────────────┐     SPI Mode 3      ┌─────────────┐
│   ESP32     │◄────────────────────►│  ICM-20948  │
│             │   1 MHz, CPOL=1     │  Accel/Gyro │
│  WebServer  │   CPHA=1            │  Mag(AK09916│
│  :80 (HTTP) │                     └─────────────┘
│  :81 (WS)   │
└──────┬──────┘
       │ WiFi AP
       │ 192.168.4.1
       ▼
┌─────────────┐
│   Browser   │
│  Canvas 2D  │
│  WebSocket  │
│  50 Hz JSON │
└─────────────┘
```

### Fluxo de dados

1. **ESP32** lê accel + gyro via SPI (registos 0x2D-0x38)
2. **ESP32** lê magnetómetro via I2C master interno (AK09916 → EXT_SLV_SENS_DATA)
3. **Complementary filter** funde accel + gyro → roll, pitch, yaw → quaternião
4. **JSON via WebSocket** a 50 Hz: `{"w","x","y","z","ax","ay","az","gx","gy","gz","mx","my","mz"}`
5. **Browser** interpola com LERP, aplica câmara isométrica, desenha setas 3D no Canvas

### Porquê SPI Mode 3?

O ICM-20948 suporta oficialmente Mode 0 e Mode 3. Contudo, alguns módulos breakout (especialmente clones) só funcionam com Mode 3 (`CPOL=1, CPHA=1`). Este sketch usa Mode 3 por compatibilidade máxima.

A library SparkFun tem `SPI_MODE0` hardcoded em `ICM_20948.cpp`. Se precisares da DMP da SparkFun, edita essa linha:

```cpp
// Em ICM_20948.cpp, procura:
_spisettings = SPISettings(_freq, MSBFIRST, SPI_MODE0);

// Troca por:
_spisettings = SPISettings(_freq, MSBFIRST, SPI_MODE3);
```

### Complementary Filter vs DMP

| | Complementary Filter | DMP (SparkFun) |
|---|---|---|
| Dependências | Nenhuma | Library SparkFun + ICM_20948_USE_DMP |
| SPI Mode | Mode 3 (compatível) | Mode 0 (pode falhar) |
| Precisão | Boa para tilt, drift lento no yaw | Excelente, fusão 9 eixos |
| Complexidade | ~30 linhas | ~14KB de firmware DMP |
| Latência | <1 ms | ~10 ms (FIFO) |

---

## Protocolo WebSocket

O ESP32 envia JSON a 50 Hz na porta 81:

```json
{
  "w": 0.99987,  "x": 0.00123,  "y": -0.01456,  "z": 0.00034,
  "ax": 0.012,   "ay": -0.031,  "az": 0.998,
  "gx": 1.2,     "gy": -0.8,    "gz": 0.3,
  "mx": 25.1,    "my": -8.4,    "mz": 42.0,
  "md": "SPI Mode3 CF"
}
```

| Campo | Unidade | Descrição |
|---|---|---|
| `w, x, y, z` | — | Quaternião de orientação (normalizado) |
| `ax, ay, az` | g | Aceleração (1g ≈ 9.81 m/s²) |
| `gx, gy, gz` | °/s | Velocidade angular |
| `mx, my, mz` | µT | Campo magnético |
| `md` | — | Modo ativo |

---

## Resolução de problemas

### Sensor não encontrado (`WHO_AM_I ≠ 0xEA`)

- Verifica alimentação a **3.3V** (não 5V!)
- Confirma as ligações SPI na tabela acima
- Alguns módulos têm pinos marcados de forma diferente — consulta o pinout do teu módulo

### HTTP não carrega a página

- Confirma que estás ligado ao WiFi `ICM20948-3D`
- Tenta `http://192.168.4.1` (não https)
- Abre o Serial Monitor — deve mostrar `[HTTP] :80 OK`

### Só aparecem 2 setas no browser

- Este sketch usa câmara isométrica — os 3 eixos estão sempre visíveis
- Se mesmo assim faltam setas, carrega no botão **Simulação** para testar

### Setas não mexem ao rodar o sensor

- Confirma que o Serial Monitor mostra valores a mudar em `A[...]` e `G[...]`
- Se os valores estão fixos, o sensor não está a comunicar — verifica fios
- Carrega no botão **Reconnect** no browser

### Drift no yaw (rotação horizontal)

- Normal no modo complementary filter sem correção do magnetómetro
- O yaw vem apenas do giroscópio e tem drift lento (~1-5°/min)
- Para yaw estável, é necessário integrar o magnetómetro no filtro (AHRS/Madgwick)

---

## Estrutura do ficheiro

```
ICM20948_Mode3.ino
├── Driver SPI (Mode 3)
│   ├── icm_write / icm_read / icm_readBytes
│   ├── icm_selectBank
│   ├── icm_init (reset, config, mag setup)
│   ├── icm_readAG (accel + gyro)
│   └── icm_readMag (magnetómetro via I2C master)
├── Complementary Filter
│   └── updateOrientation (accel+gyro → quaternião)
├── Web
│   ├── HTML/CSS/JS inline (PROGMEM)
│   ├── WebServer :80 (chunked send)
│   └── WebSocketsServer :81 (JSON broadcast)
└── WiFi
    └── AP mode (192.168.4.1)
```

---

## Próximos passos

- [ ] Integrar magnetómetro no filtro (Madgwick AHRS) para yaw estável
- [ ] Calibração do magnetómetro (hard/soft iron)
- [ ] Guardar calibração em EEPROM/NVS
- [ ] Modo STA com mDNS (`http://imu.local`)
- [ ] Exportar dados via CSV/log
- [ ] Suporte DMP via library SparkFun (requer patch SPI_MODE3)

---

## Licença

MIT — usa, modifica, e partilha livremente.
