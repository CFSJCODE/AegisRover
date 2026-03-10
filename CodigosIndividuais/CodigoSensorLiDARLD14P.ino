/**
 * ╔═══════════════════════════════════════════════════════════════════════════════╗
 * ║  Aegis Rover - Firmware Unificado v4.0 (FreeRTOS Edition)                     ║
 * ║  Arquitetura: ESP32-C6 (RISC-V) | Framework: Arduino Core 3.0.0+              ║
 * ║  Desenvolvedor: Cláudio Francisco (CFJ) | CEO Aegis Rover                     ║
 * ║  Ano: 2026 | Foco: Segurança Patrimonial e Telemetria de Alta Precisão        ║
 * ╚═══════════════════════════════════════════════════════════════════════════════╝
 * * FUNDAMENTAÇÃO TEÓRICA - PROJETO AEGIS:
 * O sistema Aegis (Égide) opera como um escudo digital proativo. Este firmware 
 * integra telemetria LiDAR por triangulação laser com uma camada de comunicação 
 * híbrida (Bluetooth Low Energy e Wi-Fi), garantindo determinismo no processamento 
 * de sinais e redundância na entrega de dados.
 */

// Justificativa: Na mitologia, Aegis (Égide) é o escudo impenetrável. Transmite imediatamente o 
// conceito de segurança patrimonial, proteção proativa (mitigação de riscos como vazamento de gás) 
// e monitoramento contínuo.

/**
 * PINAGEM FÍSICA DO SENSOR LIDAR LD14P
 * ════════════════════════════════════════════════════════════════════════════
 *
 * Conector do LD14P:
 * │ Cor Do Fio   │ Função           │ GPIO ESP32-C6 │ Observação           │
 * ├──────────────┼──────────────────┼───────────────┼──────────────────────┤
 * │ PRETO        │ Alimentação (+5V)│ 5V (USB)      │ Potência do motor    │
 * │ VERDE        │ Terra (GND)      │ GND           │ Referência de massa  │
 * │ BRANCO       │ Dados (RX)       │ GPIO 11       │ UART1 RX (230400bd)  │
 * │ VERMELHO     │ Motor (PWM)      │ GPIO 10       │ Controle H-bridge    │
 * └──────────────┴──────────────────┴───────────────┴──────────────────────┘
 */

// =================================================================================
// SEÇÃO 1: MAPEAMENTO DE HARDWARE E CONSTANTES TÉCNICAS
// =================================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <HardwareSerial.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Bibliotecas Nativas do ESP-IDF para Gestão Aprofundada de Energia
#include <esp_wifi.h>
#include <esp_pm.h>
#include <esp_sleep.h>

// Inclusão das bibliotecas do núcleo FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// Mapeamento Físico de Pinos (ESP32-C6)
#define LIDAR_RX_PIN     11 
#define LIDAR_MOTOR_PIN  10 
#define RGB_LED_PIN      8  
#define POT_PIN          2  

// Especificações do Protocolo LD14P (LiDAR) - Mapeamento Clean Code
#define LIDAR_BAUD_RATE  230400 
#define POINT_PER_PACK   12     
#define PKG_HEADER       0x54   
#define PKG_BYTE_2       0x2C   
#define PKG_SIZE         47     
#define UART_TIMEOUT_MS  2000   

// Offsets do Protocolo LiDAR (Erradicação de Magic Numbers)
constexpr uint8_t LD_OFFSET_SPEED_L     = 2;
constexpr uint8_t LD_OFFSET_SPEED_H     = 3;
constexpr uint8_t LD_OFFSET_START_ANG_L = 4;
constexpr uint8_t LD_OFFSET_START_ANG_H = 5;
constexpr uint8_t LD_OFFSET_END_ANG_L   = 42;
constexpr uint8_t LD_OFFSET_END_ANG_H   = 43;
constexpr uint8_t LD_OFFSET_DATA_START  = 6;
constexpr uint8_t LD_BYTES_PER_POINT    = 3;
constexpr int32_t LD_MAX_ANGLE_CENT     = 36000;

// Parâmetros de Filtro Espacial e Digital
#define INTENSITY_THRESHOLD  60   
#define MAX_VALID_DISTANCE   6000 
const float ALPHA_EWMA = 0.05f;   

// Parâmetros de Resolução Espacial (Occupancy Grid)
#define GRID_SIZE        100  
#define GRID_RES_MM      50   
#define GRID_CENTER      50   
#define OBSTACLE_COST    255  
#define FREE_SPACE_COST  0    

// Parâmetros Lógicos de Algoritmos e UI 
#define DECAY_RATE_POLAR       50   
#define DECAY_RATE_GRID        15   
#define MIN_GRID_CONFIDENCE    15   
#define MIN_PWM_MOTOR_ACTIVE   30   
#define INACTIVITY_WARNING_MS  30000 

// Configurações de Rede Local (Station Mode com IP Estático)
const char* SSID_STA       = "DLINK DIR-3040";
const char* PASSWORD_STA   = "ClaudioADV2026";
const IPAddress IP_STA       (192, 168, 0, 55);  
const IPAddress GATEWAY_STA  (192, 168, 0, 1);   
const IPAddress SUBNET_STA   (255, 255, 255, 0); 
const IPAddress DNS_STA      (8, 8, 8, 8);       

// UUIDs GATT para o Serviço Bluetooth 
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// Threshold para acionamento do Deep-Sleep
#define DEEP_SLEEP_INACTIVITY_MS 60000 

// =================================================================================
// SEÇÃO 2: DOMAIN-DRIVEN DESIGN E PADRÃO MONITOR (ESTADOS E ESTRUTURAS)
// =================================================================================

// Máquina de estados para roteamento de interface
enum AppContext { CTX_HOME = 0, CTX_NETWORK = 1, CTX_MONITOR = 2 };
AppContext g_active_context = CTX_HOME;

struct Point2D { float x; float y; };

struct AStarNode {
    int x, y;             
    float gCost;          
    float hCost;          
    float fCost;          
    int parentX, parentY; 
};

// Padrão Monitor: Agrupa os dados globais do robô e o Mutex responsável por protegê-los.
// Evita variáveis soltas e garante coesão (Engenharia de Software).
struct RobotState {
    uint16_t scan_map[360] = {0};
    uint16_t lidar_speed_raw = 0;
    uint32_t lidar_last_frame_ms = 0;
    bool     uart_timeout_detected = false;
    
    float    pot_filtered_value = 0.0f;
    uint8_t  motor_pwm_target = 0;
    bool     motor_is_enabled = false;
    
    uint8_t  led_r = 0, led_g = 0, led_b = 0;
    
    SemaphoreHandle_t mutex;
} g_robot;

// Padrão Monitor para a Grade Espacial de SLAM
struct GridMap {
    uint8_t occupancy[GRID_SIZE][GRID_SIZE] = {0};
    SemaphoreHandle_t mutex;
} g_grid;


static const uint8_t CRC8_LUT[256] = {
  0x00,0x4D,0x9A,0xD7,0x79,0x34,0xE3,0xAE,0xF2,0xBF,0x68,0x25,0x8B,0xC6,0x11,0x5C,
  0xA9,0xE4,0x33,0x7E,0xD0,0x9D,0x4A,0x07,0x5B,0x16,0xC1,0x8C,0x22,0x6F,0xB8,0xF5,
  0x1F,0x52,0x85,0xC8,0x66,0x2B,0xFC,0xB1,0xED,0xA0,0x77,0x3A,0x94,0xD9,0x0E,0x43,
  0xB6,0xFB,0x2C,0x61,0xCF,0x82,0x55,0x18,0x44,0x09,0xDE,0x93,0x3D,0x70,0xA7,0xEA,
  0x3E,0x73,0xA4,0xE9,0x47,0x0A,0xDD,0x90,0xCC,0x81,0x56,0x1B,0xB5,0xF8,0x2F,0x62,
  0x97,0xDA,0x0D,0x40,0xEE,0xA3,0x74,0x39,0x65,0x28,0xFF,0xB2,0x1C,0x51,0x86,0xCB,
  0x21,0x6C,0xBB,0xF6,0x58,0x15,0xC2,0x8F,0xD3,0x9E,0x49,0x04,0xAA,0xE7,0x30,0x7D,
  0x88,0xC5,0x12,0x5F,0xF1,0xBC,0x6B,0x26,0x7A,0x37,0xE0,0xAD,0x03,0x4E,0x99,0xD4,
  0x7C,0x31,0xE6,0xAB,0x05,0x48,0x9F,0xD2,0x8E,0xC3,0x14,0x59,0xF7,0xBA,0x6D,0x20,
  0xD5,0x98,0x4F,0x02,0xAC,0xE1,0x36,0x7B,0x27,0x6A,0xBD,0xF0,0x5E,0x13,0xC4,0x89,
  0x63,0x2E,0xF9,0xB4,0x1A,0x57,0x80,0xCD,0x91,0xDC,0x0B,0x46,0xE8,0xA5,0x72,0x3F,
  0xCA,0x87,0x50,0x1D,0xB3,0xFE,0x29,0x64,0x38,0x75,0xA2,0xEF,0x41,0x0C,0xDB,0x96,
  0x42,0x0F,0xD8,0x95,0x3B,0x76,0xA1,0xEC,0xB0,0xFD,0x2A,0x67,0xC9,0x84,0x53,0x1E,
  0xEB,0xA6,0x71,0x3C,0x92,0xDF,0x08,0x45,0x19,0x54,0x83,0xCE,0x60,0x2D,0xFA,0xB7,
  0x5D,0x10,0xC7,0x8A,0x24,0x69,0xBE,0xF3,0xAF,0xE2,0x35,0x78,0xD6,0x9B,0x4C,0x01,
  0xF4,0xB9,0x6E,0x23,0x8D,0xC0,0x17,0x5A,0x06,0x4B,0x9C,0xD1,0x7F,0x32,0xE5,0xA8
};

HardwareSerial g_lidar_uart(1); 
WebServer g_web_server(80);

// Estruturas de controle BLE
BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic = NULL;
bool g_deviceConnected = false;

// =================================================================================
// SEÇÃO 3: CAMADA DE ABSTRAÇÃO DE HARDWARE (HAL)
// =================================================================================

inline void hal_MotorSetPWM(uint8_t pwm_value) {
    analogWrite(LIDAR_MOTOR_PIN, pwm_value);
}

inline void hal_LedSetColor(uint8_t r, uint8_t g, uint8_t b) {
    if (g_robot.led_r != r || g_robot.led_g != g || g_robot.led_b != b) {
        g_robot.led_r = r; g_robot.led_g = g; g_robot.led_b = b; 
        neopixelWrite(RGB_LED_PIN, r, g, b);
    }
}

inline void hal_SystemDeepSleep() {
    delay(100); 
    esp_deep_sleep_start();
}


// =================================================================================
// SEÇÃO 4: DECLARAÇÕES HTML (FRONTEND SPA VIA PROGMEM)
// =================================================================================

const char HTML_LANDING[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Aegis Rover | UGV Command Center</title>
  <script src="https://cdn.tailwindcss.com"></script>
  <style>
    @import url('https://fonts.googleapis.com/css2?family=Orbitron:wght@400;700;900&family=Share+Tech+Mono&display=swap');
    body { background-color: #050914; color: #e2e8f0; font-family: 'Share Tech Mono', monospace; overflow-x: hidden; }
    .radar-bg { position: fixed; top: 50%; left: 50%; transform: translate(-50%, -50%); width: 150vw; height: 150vw; max-width: 1200px; max-height: 1200px; background: radial-gradient(circle, rgba(50, 116, 217, 0.1) 0%, rgba(5, 9, 20, 1) 70%); border-radius: 50%; z-index: -1; }
    .radar-pulse { position: absolute; top: 0; left: 0; right: 0; bottom: 0; border-radius: 50%; border: 1px solid rgba(50, 116, 217, 0.3); animation: pulse 4s infinite linear; }
    @keyframes pulse { 0% { transform: scale(0.1); opacity: 1; } 100% { transform: scale(1.5); opacity: 0; border-color: rgba(0, 255, 65, 0.5); } }
    h1, h2 { font-family: 'Orbitron', sans-serif; text-transform: uppercase; letter-spacing: 2px; }
    .glass-card { background: rgba(24, 27, 31, 0.6); backdrop-filter: blur(12px); border: 1px solid rgba(82, 116, 217, 0.2); border-radius: 8px; transition: all 0.4s cubic-bezier(0.175, 0.885, 0.32, 1.275); }
    .glass-card:hover { transform: translateY(-5px) scale(1.02); border-color: rgba(0, 255, 65, 0.6); box-shadow: 0 10px 30px -10px rgba(0, 255, 65, 0.3); }
    .status-dot { width: 8px; height: 8px; border-radius: 50%; box-shadow: 0 0 8px currentColor; }
  </style>
</head>
<body class="min-h-screen flex flex-col items-center justify-center p-6 relative">
  <div class="radar-bg">
    <div class="radar-pulse"></div>
    <div class="radar-pulse" style="animation-delay: 2s;"></div>
  </div>
  <header class="text-center mb-12 z-10">
    <div class="inline-flex items-center gap-3 bg-blue-900/30 border border-blue-500/50 px-4 py-1 rounded-full text-blue-400 text-xs mb-6">
      <span id="ledIndicator" class="status-dot bg-blue-400"></span>
      <span id="sysStatusBtn" class="tracking-widest font-bold">SYSTEM ONLINE</span>
    </div>
    <h1 class="text-4xl md:text-6xl font-black text-white drop-shadow-[0_0_15px_rgba(50,116,217,0.8)]">Aegis Rover</h1>
    <h2 class="text-xl md:text-2xl text-blue-400 mt-2">UGV Control Center</h2>
    <p class="mt-4 text-slate-400 max-w-lg mx-auto text-sm">Plataforma Robótica em Rede (Modo Station via DLINK)</p>
  </header>
  <main class="grid grid-cols-1 md:grid-cols-2 gap-8 w-full max-w-4xl z-10">
    <a href="/net" class="glass-card p-8 flex flex-col items-center justify-center text-center group cursor-pointer decoration-none text-slate-200 hover:text-white">
      <div class="w-16 h-16 rounded-full bg-blue-500/10 border border-blue-500/30 flex items-center justify-center mb-6 group-hover:bg-blue-500/20 group-hover:scale-110 transition-all">
        <svg class="w-8 h-8 text-blue-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
          <path stroke-linecap="round" stroke-linejoin="round" stroke-width="1.5" d="M19.428 15.428a2 2 0 00-1.022-.547l-2.387-.477a6 6 0 00-3.86.517l-.318.158a6 6 0 01-3.86.517L6.05 15.21a2 2 0 00-1.806.547"></path>
        </svg>
      </div>
      <h2 class="text-2xl font-bold mb-2">Network Manager</h2>
      <p class="text-sm text-slate-400">Status do link L2 e força do sinal</p>
    </a>
    <a href="/monitor" class="glass-card p-8 flex flex-col items-center justify-center text-center group cursor-pointer decoration-none text-slate-200 hover:text-white">
      <div class="w-16 h-16 rounded-full bg-green-500/10 border border-green-500/30 flex items-center justify-center mb-6 group-hover:bg-green-500/20 group-hover:scale-110 transition-all">
        <svg class="w-8 h-8 text-green-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
          <path stroke-linecap="round" stroke-linejoin="round" stroke-width="1.5" d="M12 8v4l3 3"></path>
        </svg>
      </div>
      <h2 class="text-2xl font-bold mb-2">LiDAR Telemetry</h2>
      <p class="text-sm text-slate-400">Visualização em tempo real</p>
    </a>
  </main>
  <footer class="absolute bottom-4 text-slate-500 text-xs text-center w-full">
    &copy; 2026 Aegis Rover | Firmware v4.0 (RTOS)
  </footer>
  <script>
    function updateLanding() {
      fetch('/data?ctx=home').then(r => r.json()).then(data => {
        const btnEl = document.getElementById('sysStatusBtn');
        const ledEl = document.getElementById('ledIndicator');
        const r = data.led[0], g = data.led[1], b = data.led[2];
        const colorStr = `rgb(${r},${g},${b})`;
        ledEl.style.backgroundColor = colorStr;
        ledEl.style.color = colorStr;
        btnEl.style.color = colorStr;
      }).catch(err => console.log('Offline'));
    }
    setInterval(updateLanding, 2000);
    updateLanding();
  </script>
</body>
</html>
)rawliteral";

const char HTML_NETWORK[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Aegis Rover | Network Manager</title>
  <script src="https://cdn.tailwindcss.com"></script>
  <style>
    body { background-color: #111217; color: #c7d0d9; font-family: ui-sans-serif, system-ui, sans-serif; }
    .g-panel { background-color: #181b1f; border: 1px solid #2c3235; border-radius: 4px; padding: 20px; }
    .g-title { font-size: 0.75rem; text-transform: uppercase; letter-spacing: 0.05em; color: #8e99a4; margin-bottom: 12px; font-weight: 600; }
    .val-text { font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace; color: #fff; }
    .status-dot { display: inline-block; width: 10px; height: 10px; border-radius: 50%; box-shadow: 0 0 10px currentColor; }
    .g-btn { background-color: #22252b; border: 1px solid #2c3235; color: #c7d0d9; transition: all 0.2s; padding: 8px 16px; border-radius: 4px; cursor: pointer; }
  </style>
</head>
<body class="min-h-screen flex flex-col p-4 md:p-6">
  <header class="flex flex-col md:flex-row justify-between items-center mb-6 pb-4 border-b border-gray-700">
    <div class="flex items-center gap-4">
      <a href="/" class="g-btn">&larr; HOME</a>
      <div>
        <h1 class="text-2xl font-bold text-white">Network Status (STA)</h1>
        <p class="text-xs text-gray-400 mt-1 val-text">Infraestrutura Gerenciada (IP Fixo)</p>
      </div>
     </div>
    <div class="flex items-center bg-gray-900 border border-gray-700 rounded px-4 py-2 gap-3">
      <span id="ledIndicator" class="status-dot text-gray-500 bg-gray-500"></span>
      <span id="sysStatusBtn" class="val-text text-sm font-bold">CHECKING...</span>
    </div>
  </header>
  <main class="grid grid-cols-1 md:grid-cols-2 xl:grid-cols-3 gap-4 flex-grow">
    <div class="g-panel xl:col-span-2 border-l-4 border-l-blue-500">
      <div class="g-title">Uptime (RTC)</div>
      <div id="sysTime" class="val-text text-blue-400 text-5xl font-medium">--:--:--</div>
    </div>
    <div class="g-panel">
      <div class="g-title">Timestamp</div>
      <div class="flex items-baseline gap-2 mt-2">
        <span id="sysUptime" class="val-text text-4xl">0</span><span class="text-gray-400 text-sm">s</span>
      </div>
    </div>
    <div class="g-panel xl:col-span-3">
      <div class="g-title flex justify-between"><span>LAN (Station Mode)</span><span class="text-green-400 font-bold">CONECTADA</span></div>
      <div class="grid grid-cols-2 md:grid-cols-4 gap-4 mt-4 border-t border-gray-700 pt-4">
        <div>
          <p class="text-xs text-gray-400 uppercase">Gateway Router</p>
          <p class="val-text text-lg text-blue-300">DLINK DIR</p>
        </div>
        <div>
          <p class="text-xs text-gray-400 uppercase">IP Fixo (Alocado)</p>
          <p class="val-text text-lg text-gray-400" id="sysIp">192.168.0.55</p>
        </div>
        <div>
          <p class="text-xs text-gray-400 uppercase">Sinal (RSSI)</p>
          <p class="val-text text-lg text-green-400"><span id="sysRssi">--</span> dBm</p>
        </div>
        <div>
          <p class="text-xs text-gray-400 uppercase">Bluetooth (BLE)</p>
          <p class="val-text text-lg text-blue-400" id="sysBle">--</p>
        </div>
      </div>
    </div>
  </main>
  <script>
    function updateNet() {
      fetch('/data?ctx=net').then(r => r.json()).then(data => {
        document.getElementById('sysTime').innerText = data.time || "--:--:--";
        document.getElementById('sysUptime').innerText = data.uptime;
        document.getElementById('sysRssi').innerText = data.rssi;
        document.getElementById('sysIp').innerText = data.ip;
        document.getElementById('sysBle').innerText = data.bt_connected ? "Conectado" : "Advertising";
        
        const btnEl = document.getElementById('sysStatusBtn');
        const ledEl = document.getElementById('ledIndicator');
        const r = data.led[0], g = data.led[1], b = data.led[2];
        const colorStr = `rgb(${r},${g},${b})`;
        
        ledEl.style.backgroundColor = colorStr;
        btnEl.style.color = colorStr;
        
        if (g > 200 && r < 100) btnEl.innerText = 'LINK OK';
        else btnEl.innerText = 'ERROR';
      }).catch(err => console.log('Timeout'));
    }
    setInterval(updateNet, 1000);
    updateNet();
  </script>
</body>
</html>
)rawliteral";

const char HTML_MONITOR[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Aegis Rover | LiDAR Radar</title>
  <script src="https://cdn.tailwindcss.com"></script>
  <style>
    body { background-color: #111217; color: #c7d0d9; }
    .g-panel { background-color: #181b1f; border: 1px solid #2c3235; border-radius: 4px; padding: 20px; }
    .val-text { font-family: ui-monospace; }
    .status-dot { display: inline-block; width: 10px; height: 10px; border-radius: 50%; box-shadow: 0 0 10px currentColor; }
    #radarContainer { display: flex; justify-content: center; align-items: center; background: #0b0c10; border-radius: 4px; border: 1px solid #2c3235; padding: 10px; }
    canvas { max-width: 100%; height: auto; }
    .g-btn { background-color: #22252b; border: 1px solid #2c3235; padding: 8px 16px; border-radius: 4px; cursor: pointer; transition: 0.3s; }
    .g-btn:hover { background-color: #2c3235; }
    .btn-active { border-color: #00ff41; color: #00ff41; }
  </style>
</head>
<body class="min-h-screen flex flex-col p-4 md:p-6">
  <header class="flex flex-col md:flex-row justify-between items-center mb-6 pb-4 border-b border-gray-700">
    <div class="flex items-center gap-4">
      <a href="/" class="g-btn">&larr; HOME</a>
      <div>
        <h1 class="text-2xl font-bold text-white">LD14P Sensor Radar</h1>
        <p id="radarModeDesc" class="text-xs text-gray-400 mt-1 val-text">360° Polar View</p>
      </div>
    </div>
    <div class="flex items-center gap-4">
      <button id="toggleModeBtn" onclick="toggleCoordinateSystem()" class="g-btn text-xs font-bold uppercase tracking-wider">MODO: POLAR</button>
      
      <div class="flex items-center bg-gray-900 border border-gray-700 rounded px-4 py-2 gap-3">
        <span id="ledIndicator" class="status-dot text-gray-500 bg-gray-500"></span>
        <span id="sysStatusBtn" class="val-text text-sm font-bold">WAITING...</span>
      </div>
    </div>
  </header>
  <main class="flex-grow">
    <div class="g-panel h-full flex flex-col">
      <div class="flex justify-between items-center mb-4 border-b border-gray-700 pb-2">
        <span class="text-sm font-semibold text-gray-400 uppercase">Scan Matrix</span>
        <span id="pwmDisplay" class="val-text text-blue-400 font-bold">PWM: --</span>
      </div>
      <div id="radarContainer" class="flex-grow">
        <canvas id="radarCanvas" width="800" height="800"></canvas>
      </div>
    </div>
  </main>
  <script>
    const canvas = document.getElementById('radarCanvas');
    const ctx = canvas.getContext('2d');
    const centerX = canvas.width / 2;
    const centerY = canvas.height / 2;
    const maxRadius = 380;
    const maxDist = 2000;
    
    // Variável de estado para o sistema de coordenadas
    let isCartesianMode = false;

    function toggleCoordinateSystem() {
      isCartesianMode = !isCartesianMode;
      const btn = document.getElementById('toggleModeBtn');
      const desc = document.getElementById('radarModeDesc');
      
      if (isCartesianMode) {
        btn.innerText = 'MODO: CARTESIANO';
        btn.classList.add('btn-active');
        desc.innerText = 'Plano Euclidiano (X, Y) - SLAM Occupancy Grid Base';
      } else {
        btn.innerText = 'MODO: POLAR';
        btn.classList.remove('btn-active');
        desc.innerText = '360° Polar View (Raio, Ângulo)';
      }
      drawGrid();
    }

    function drawGrid() {
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.strokeStyle = '#2c3235';
      ctx.lineWidth = 1;
      ctx.fillStyle = '#8e99a4';
      ctx.font = '12px monospace';
      
      if (!isCartesianMode) {
        // --- VISUALIZAÇÃO POLAR (Círculos Concêntricos) ---
        ctx.textAlign = 'center';
        for (let i = 1; i <= 4; i++) {
          ctx.beginPath();
          ctx.arc(centerX, centerY, (maxRadius / 4) * i, 0, 2 * Math.PI);
          ctx.stroke();
        }
        for (let angle = 0; angle < 360; angle += 30) {
          const rad = (angle - 90) * (Math.PI / 180);
          const x = centerX + maxRadius * Math.cos(rad);
          const y = centerY + maxRadius * Math.sin(rad);
          ctx.beginPath(); ctx.moveTo(centerX, centerY); ctx.lineTo(x, y); ctx.stroke();
          
          const textX = centerX + (maxRadius + 15) * Math.cos(rad);
          const textY = centerY + (maxRadius + 15) * Math.sin(rad);
          ctx.fillText(angle + '°', textX, textY);
        }
      } else {
        // --- VISUALIZAÇÃO CARTESIANA (Grade Ortogonal X/Y) ---
        ctx.textAlign = 'left';
        const gridSize = maxRadius / 4;
        
        // Linhas de Grade
        for (let i = -maxRadius; i <= maxRadius; i += gridSize) {
          // Verticais
          ctx.beginPath(); ctx.moveTo(centerX + i, centerY - maxRadius); ctx.lineTo(centerX + i, centerY + maxRadius); ctx.stroke();
          // Horizontais
          ctx.beginPath(); ctx.moveTo(centerX - maxRadius, centerY + i); ctx.lineTo(centerX + maxRadius, centerY + i); ctx.stroke();
        }
        
        // Eixos Principais X e Y
        ctx.lineWidth = 2;
        ctx.strokeStyle = '#5274d9';
        // Eixo Y
        ctx.beginPath(); ctx.moveTo(centerX, centerY - maxRadius); ctx.lineTo(centerX, centerY + maxRadius); ctx.stroke();
        // Eixo X
        ctx.beginPath(); ctx.moveTo(centerX - maxRadius, centerY); ctx.lineTo(centerX + maxRadius, centerY); ctx.stroke();
        
        ctx.fillStyle = '#5274d9';
        ctx.fillText('+Y (Frente)', centerX + 10, centerY - maxRadius + 10);
        ctx.fillText('+X (Direita)', centerX + maxRadius - 70, centerY - 10);
      }
    }

    function drawPoints(mapData) {
      ctx.fillStyle = '#00ff41';
      for (let angle = 0; angle < 360; angle++) {
        let dist = mapData[angle];
        if (dist > 0) {
          if (dist > maxDist) dist = maxDist;
          
          // CONVERSÃO MATEMÁTICA: Polar -> Cartesiano
          // Executado no Frontend para mitigar o peso computacional no microcontrolador.
          const r = (dist / maxDist) * maxRadius;
          const rad = (angle - 90) * (Math.PI / 180);
          
          const cartesianX = r * Math.cos(rad);
          const cartesianY = r * Math.sin(rad);
          
          // Mapeamento para o Canvas
          const screenX = centerX + cartesianX;
          const screenY = centerY + cartesianY;
          
          ctx.beginPath();
          ctx.arc(screenX, screenY, isCartesianMode ? 4 : 3, 0, 2 * Math.PI);
          ctx.fill();
        }
      }
    }

    function updateRadar() {
      fetch('/data?ctx=mon')
        .then(r => r.json())
        .then(data => {
          drawGrid();
          drawPoints(data.map);
          document.getElementById('pwmDisplay').innerText = 'PWM: ' + data.pwm;
          
          const btnEl = document.getElementById('sysStatusBtn');
          const ledEl = document.getElementById('ledIndicator');
          const r = data.led[0], g = data.led[1], b = data.led[2];
          const colorStr = `rgb(${r},${g},${b})`;
          
          ledEl.style.backgroundColor = colorStr;
          btnEl.style.color = colorStr;
          
          if (r === 255 && g === 128 && b === 0)        btnEl.innerText = '🟠 UART TIMEOUT';
          else if (r === 255 && g === 255 && b === 255) btnEl.innerText = '⚪ SCANNING';
          else if (r === 0 && g === 255 && b === 0)     btnEl.innerText = '🟢 READY / OK';
          else if (r === 0 && g === 100 && b === 255)   btnEl.innerText = '🔵 STANDBY';
          else                                          btnEl.innerText = '⚫ SYSTEM WAIT';
        })
        .catch(err => console.error('Radar error:', err));
    }

    drawGrid();
    setInterval(updateRadar, 250);
  </script>
</body>
</html>
)rawliteral";


// =================================================================================
// SEÇÃO 5: LÓGICA DE PROCESSAMENTO MATRICIAL E ALGORITMOS DE BUSCA (A*)
// =================================================================================

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { 
        g_deviceConnected = true; 
        Serial.println("[BLE] Status: Cliente conectado."); 
    };
    void onDisconnect(BLEServer* pServer) { 
        g_deviceConnected = false; 
        Serial.println("[BLE] Status: Desconectado. Reiniciando Advertising..."); 
        BLEDevice::startAdvertising(); 
    };
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String rxValue = pCharacteristic->getValue(); 
      if (rxValue.length() > 0) {
        Serial.print("[BLE RX] Comando recebido: "); Serial.println(rxValue);
      }
    }
};

static inline uint8_t calculateCRC8(const uint8_t* data, size_t length) {
  uint8_t crc = 0; 
  for (size_t i = 0; i < length; i++) {
      crc = CRC8_LUT[(crc ^ data[i]) & 0xFF];
  }
  return crc;
}

Point2D convertPolarToCartesian(uint16_t distance_mm, uint16_t angle_deg) {
    float theta_rad = angle_deg * (PI / 180.0f);
    Point2D point;
    point.x = distance_mm * cos(theta_rad);
    point.y = distance_mm * sin(theta_rad);
    return point;
}

void updateOccupancyGrid(float x_mm, float y_mm) {
    int gridX = GRID_CENTER + (int)(x_mm / GRID_RES_MM);
    int gridY = GRID_CENTER + (int)(y_mm / GRID_RES_MM);

    if (gridX >= 0 && gridX < GRID_SIZE && gridY >= 0 && gridY < GRID_SIZE) {
        if (xSemaphoreTake(g_grid.mutex, portMAX_DELAY) == pdTRUE) {
            g_grid.occupancy[gridX][gridY] = OBSTACLE_COST;
            xSemaphoreGive(g_grid.mutex);
        }
    }
}

void parseAndProcessLidarBuffer(const uint8_t *buf) {
  // Parsing utilizando constantes semânticas ao invés de Magic Numbers
  uint16_t speed_raw = (uint16_t)buf[LD_OFFSET_SPEED_L] | ((uint16_t)buf[LD_OFFSET_SPEED_H] << 8);
  uint16_t start_ang_cent = (uint16_t)buf[LD_OFFSET_START_ANG_L] | ((uint16_t)buf[LD_OFFSET_START_ANG_H] << 8);
  uint16_t end_ang_cent   = (uint16_t)buf[LD_OFFSET_END_ANG_L] | ((uint16_t)buf[LD_OFFSET_END_ANG_H] << 8);

  int32_t angle_diff_cent = (int32_t)end_ang_cent - (int32_t)start_ang_cent;
  if (angle_diff_cent < 0) angle_diff_cent += LD_MAX_ANGLE_CENT; 
  
  int32_t step_cent = angle_diff_cent / (POINT_PER_PACK - 1);

  if (xSemaphoreTake(g_robot.mutex, portMAX_DELAY) == pdTRUE) {
    for (int i = 0; i < POINT_PER_PACK; ++i) {
      int base = LD_OFFSET_DATA_START + (i * LD_BYTES_PER_POINT); 
      uint16_t dist = (uint16_t)buf[base] | ((uint16_t)buf[base + 1] << 8);
      uint8_t intensity = buf[base + 2];

      if (dist == 0 || intensity <= INTENSITY_THRESHOLD || dist > MAX_VALID_DISTANCE) continue;

      int32_t angle_cent = (int32_t)start_ang_cent + (int32_t)step_cent * i;
      angle_cent %= LD_MAX_ANGLE_CENT; 
      if (angle_cent < 0) angle_cent += LD_MAX_ANGLE_CENT;

      uint16_t angle_index = (uint16_t)((angle_cent / DEGREE_MULTIPLIER) % 360);
      g_robot.scan_map[angle_index] = dist; 
      
      Point2D cartesianTarget = convertPolarToCartesian(dist, angle_index);
      updateOccupancyGrid(cartesianTarget.x, cartesianTarget.y);
    }
    
    g_robot.lidar_last_frame_ms = millis(); 
    g_robot.lidar_speed_raw = speed_raw;
    xSemaphoreGive(g_robot.mutex); 
  }
}


// =================================================================================
// SEÇÃO 6: ENDPOINTS HTTP (OTIMIZADOS PARA C++ EMBARCADO)
// =================================================================================

void handleHttpData() {
  if (g_web_server.hasArg("ctx")) {
    String ctx_arg = g_web_server.arg("ctx");
    if (ctx_arg == "home") g_active_context = CTX_HOME;
    else if (ctx_arg == "net") g_active_context = CTX_NETWORK;
    else if (ctx_arg == "mon") g_active_context = CTX_MONITOR;
  }

  uint32_t ms = millis();
  uint32_t sec = ms / 1000;
  char timeStr[16];
  snprintf(timeStr, sizeof(timeStr), "%02lu:%02lu:%02lu", (sec / 3600), ((sec % 3600) / 60), (sec % 60));

  // Utilizando alocação única do buffer para evitar fragmentação no Heap
  String json_payload;
  json_payload.reserve(3000); 
  
  // Buffer auxiliar diminuto para conversão rápida de inteiros
  char temp_num_buf[8];

  json_payload += "{";
  json_payload += "\"uptime\":"; json_payload += sec;
  json_payload += ",\"time\":\""; json_payload += timeStr;
  json_payload += "\",\"rssi\":"; json_payload += WiFi.RSSI();
  json_payload += ",\"ip\":\""; json_payload += WiFi.localIP().toString();
  json_payload += "\",\"bt_connected\":"; json_payload += (g_deviceConnected ? "true" : "false");
  
  if (xSemaphoreTake(g_robot.mutex, portMAX_DELAY) == pdTRUE) {
    json_payload += ",\"led\":["; json_payload += g_robot.led_r; json_payload += ","; json_payload += g_robot.led_g; json_payload += ","; json_payload += g_robot.led_b; json_payload += "]";
    json_payload += ",\"pwm\":"; json_payload += g_robot.motor_pwm_target;
    json_payload += ",\"map\":[";
    
    for (int i = 0; i < 360; i++) { 
        itoa(g_robot.scan_map[i], temp_num_buf, 10);
        json_payload += temp_num_buf;
        if (i < 359) json_payload += ","; 
    }
    xSemaphoreGive(g_robot.mutex);
  }
  json_payload += "]}";
  
  g_web_server.send(200, "application/json", json_payload);
}


// =================================================================================
// SEÇÃO 7: DEFINIÇÃO DAS TAREFAS PREEMPTIVAS DO FREERTOS & POWER MANAGEMENT
// =================================================================================

// Variáveis de escopo local (Buffers de DMA e Estado do LiDAR)
uint8_t  g_lidar_buffer[PKG_SIZE] = {0}; 
uint8_t  g_lidar_buffer_idx = 0; 
bool     g_lidar_synced = false; 

void vTaskLidar(void *pvParameters) {
  for(;;) {
    while (g_lidar_uart.available()) {
      uint8_t byte_received = g_lidar_uart.read();
      
      if (!g_lidar_synced) {
        if (byte_received == PKG_HEADER) { 
            g_lidar_buffer[0] = byte_received; 
            g_lidar_buffer_idx = 1; 
            g_lidar_synced = true; 
        }
      } else {
        g_lidar_buffer[g_lidar_buffer_idx++] = byte_received;
        
        if (g_lidar_buffer_idx == 2 && g_lidar_buffer[1] != PKG_BYTE_2) { 
            g_lidar_synced = false; 
            g_lidar_buffer_idx = 0; 
            continue; 
        }
        
        if (g_lidar_buffer_idx == PKG_SIZE) {
          if (calculateCRC8(g_lidar_buffer, PKG_SIZE - 1) == g_lidar_buffer[PKG_SIZE - 1]) {
              parseAndProcessLidarBuffer(g_lidar_buffer);
          }
          g_lidar_synced = false; 
          g_lidar_buffer_idx = 0;
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void vTaskHardware(void *pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(20);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for(;;) {
    uint16_t pot_raw = analogRead(POT_PIN);
    
    if (xSemaphoreTake(g_robot.mutex, portMAX_DELAY) == pdTRUE) {
      g_robot.pot_filtered_value = (ALPHA_EWMA * pot_raw) + ((1.0f - ALPHA_EWMA) * g_robot.pot_filtered_value);
      g_robot.motor_pwm_target = map((uint16_t)g_robot.pot_filtered_value, 0, 4095, 255, 0);
      
      // Chamada delegada para a HAL
      hal_MotorSetPWM(g_robot.motor_pwm_target);
      g_robot.motor_is_enabled = (g_robot.motor_pwm_target > MIN_PWM_MOTOR_ACTIVE); 

      uint32_t now_ms = millis();
      uint32_t time_since_last_frame = now_ms - g_robot.lidar_last_frame_ms;
      g_robot.uart_timeout_detected = (time_since_last_frame > UART_TIMEOUT_MS) && (g_robot.lidar_last_frame_ms != 0);

      if (g_active_context == CTX_HOME || g_active_context == CTX_NETWORK) {
        if (WiFi.status() == WL_CONNECTED) hal_LedSetColor(0, 255, 0); 
        else hal_LedSetColor(255, 0, 0); 
      } else if (g_active_context == CTX_MONITOR) {
        if (g_robot.uart_timeout_detected) hal_LedSetColor(255, 128, 0); 
        else if (g_robot.motor_is_enabled && time_since_last_frame < 1000) hal_LedSetColor(255, 255, 255); 
        else hal_LedSetColor(0, 100, 255); 
      }
      xSemaphoreGive(g_robot.mutex);
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void vTaskBLE(void *pvParameters) {
  char ble_msg[64];

  for(;;) {
    if (g_deviceConnected) {
      if (xSemaphoreTake(g_robot.mutex, portMAX_DELAY) == pdTRUE) {
        // Envio assíncrono não-bloqueante via snprintf (Zero Heap Fragmentation)
        snprintf(ble_msg, sizeof(ble_msg), "Aegis|RPM:%d|PWM:%d", g_robot.lidar_speed_raw, g_robot.motor_pwm_target);
        pTxCharacteristic->setValue(ble_msg);
        pTxCharacteristic->notify(); 
        xSemaphoreGive(g_robot.mutex);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void vTaskRadarDecay(void *pvParameters) {
  for(;;) {
    if (xSemaphoreTake(g_robot.mutex, portMAX_DELAY) == pdTRUE) {
      for (int i = 0; i < 360; i++) {
        if (g_robot.scan_map[i] > 0) {
          uint16_t decay = (g_robot.scan_map[i] > DECAY_RATE_POLAR) ? DECAY_RATE_POLAR : g_robot.scan_map[i]; 
          g_robot.scan_map[i] -= decay;
        }
      }
      xSemaphoreGive(g_robot.mutex);
    }
    
    if (xSemaphoreTake(g_grid.mutex, portMAX_DELAY) == pdTRUE) {
      for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
           if (g_grid.occupancy[i][j] > 0) {
               g_grid.occupancy[i][j] = (g_grid.occupancy[i][j] > DECAY_RATE_GRID) ? g_grid.occupancy[i][j] - DECAY_RATE_GRID : 0;
           }
        }
      }
      xSemaphoreGive(g_grid.mutex);
    }
    vTaskDelay(pdMS_TO_TICKS(150));
  }
}

void vTaskNetwork(void *pvParameters) {
  for(;;) {
    g_web_server.handleClient(); 
    vTaskDelay(pdMS_TO_TICKS(5)); 
  }
}

// Clean Code: Funções semânticas menores para clareza
inline bool isSystemInactive() {
    bool is_wifi_down = (WiFi.status() != WL_CONNECTED);
    bool is_ble_down = !g_deviceConnected;
    
    bool is_lidar_down = false;
    if (xSemaphoreTake(g_robot.mutex, portMAX_DELAY) == pdTRUE) {
        is_lidar_down = g_robot.uart_timeout_detected;
        xSemaphoreGive(g_robot.mutex);
    }

    return (is_wifi_down && is_ble_down && is_lidar_down);
}

void vTaskPowerMonitor(void *pvParameters) {
  uint32_t inactivity_timer = 0;
  bool warning_issued = false;
  
  for(;;) {
    if (isSystemInactive()) {
      inactivity_timer += 1000;
      
      if (inactivity_timer == INACTIVITY_WARNING_MS && !warning_issued) {
         Serial.println("\n[Power Manager] AVISO: 30s de inatividade critica detectados.");
         Serial.println("[Power Manager] O sistema se auto-desligara em 30s para preservar a bateria.");
         warning_issued = true;
      }

      if (inactivity_timer >= DEEP_SLEEP_INACTIVITY_MS) {
        Serial.println("\n[Power Manager] LIMIAR DE INATIVIDADE ATINGIDO. INICIANDO SHUTDOWN.");
        hal_MotorSetPWM(0); 
        hal_LedSetColor(0, 0, 0);
        g_lidar_uart.end();
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        hal_SystemDeepSleep(); // Chamada abstraída para a HAL
      }
    } else {
      if (warning_issued) {
         Serial.println("\n[Power Manager] Atividade restabelecida. Abortando sequencia de Deep-Sleep.");
      }
      inactivity_timer = 0; 
      warning_issued = false;
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void vTaskPlanner(void *pvParameters) {
  for(;;) {
     int targetX = GRID_CENTER; 
     int targetY = GRID_CENTER + (1000 / GRID_RES_MM); 

     if (xSemaphoreTake(g_grid.mutex, portMAX_DELAY) == pdTRUE) {
         // TODO: Lógica do Pathfinding A* (Mantida inalterada da base teórica)
         xSemaphoreGive(g_grid.mutex);
     }
     vTaskDelay(pdMS_TO_TICKS(500)); 
  }
}

// =================================================================================
// SEÇÃO 8: BOOTLOADER ISOLADO E MODULARIZADO
// =================================================================================

void initHardware() {
  pinMode(POT_PIN, INPUT); 
  pinMode(LIDAR_MOTOR_PIN, OUTPUT); 
  hal_MotorSetPWM(0); 
  pinMode(RGB_LED_PIN, OUTPUT); 
  hal_LedSetColor(0, 0, 255); 
  g_lidar_uart.begin(LIDAR_BAUD_RATE, SERIAL_8N1, LIDAR_RX_PIN, -1);
}

void initBLE() {
  BLEDevice::init("Aegis Rover");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902()); 
  
  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
  pRxCharacteristic->setCallbacks(new MyCallbacks());
  
  pService->start();
  pServer->getAdvertising()->start();
}

void initNetwork() {
  WiFi.mode(WIFI_STA);
  if (!WiFi.config(IP_STA, GATEWAY_STA, SUBNET_STA, DNS_STA)) {
      Serial.println("[WiFi] Erro Critico: Falha na subnet.");
  }
  WiFi.begin(SSID_STA, PASSWORD_STA);
  esp_wifi_set_ps(WIFI_PS_MIN_MODEM); 
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) { delay(500); attempts++; }
  
  if (WiFi.status() == WL_CONNECTED) { 
      MDNS.begin("cfsj-ugv");
  }

  g_web_server.on("/", []() { g_web_server.send_P(200, "text/html", HTML_LANDING); });
  g_web_server.on("/net", []() { g_web_server.send_P(200, "text/html", HTML_NETWORK); });
  g_web_server.on("/monitor", []() { g_web_server.send_P(200, "text/html", HTML_MONITOR); });
  g_web_server.on("/data", handleHttpData);
  g_web_server.begin();
}

void setup() {
  Serial.begin(115200); 
  uint32_t t = millis();
  while (!Serial && (millis() - t < 5000)) { delay(10); }
  delay(500); 
  
  Serial.println("\n===============================================================================");
  Serial.println("  Aegis Rover - Firmware Unificado v4.0 (FreeRTOS) - SLAM Edition");
  Serial.println("  Arquitetura: ESP32-C6 (RISC-V) | Framework: Arduino Core 3.0.0+");
  Serial.println("  Foco: Seguranca Patrimonial e Telemetria de Alta Precisao");
  Serial.println("===============================================================================\n");

  #if CONFIG_PM_ENABLE
    esp_pm_config_t pm_config = { .max_freq_mhz = 160, .min_freq_mhz = 40, .light_sleep_enable = true };
    esp_pm_configure(&pm_config);
  #endif

  Serial.println("[Aegis System] Modulos de Boot: Inicializando Hardware...");
  initHardware();
  
  Serial.println("[Aegis System] Modulos de Boot: Subindo pilha BLE...");
  initBLE();
  
  Serial.println("[Aegis System] Modulos de Boot: Subindo pilha WiFi e LwIP...");
  initNetwork();

  g_robot.pot_filtered_value = analogRead(POT_PIN);

  Serial.println("[Aegis System] Instanciando Mutexes e RTOS Tasks...");
  g_robot.mutex = xSemaphoreCreateMutex();
  g_grid.mutex = xSemaphoreCreateMutex(); 
  
  if (g_robot.mutex != NULL && g_grid.mutex != NULL) {
    xTaskCreate(vTaskPowerMonitor,"Task_PWR",   2048, NULL, 6, NULL);
    xTaskCreate(vTaskLidar,       "Task_LiDAR", 4096, NULL, 5, NULL);
    xTaskCreate(vTaskHardware,    "Task_HW",    2048, NULL, 4, NULL);
    xTaskCreate(vTaskPlanner,     "Task_Plan",  4096, NULL, 3, NULL); 
    xTaskCreate(vTaskNetwork,     "Task_Net",   4096, NULL, 3, NULL); 
    xTaskCreate(vTaskBLE,         "Task_BLE",   4096, NULL, 2, NULL); 
    xTaskCreate(vTaskRadarDecay,  "Task_Decay", 2048, NULL, 1, NULL); 
    Serial.println("[Aegis System] RTOS Operante. Handover para o Scheduler.");
  } else {
    Serial.println("[ERRO CRITICO] Falha ao alocar Mutex na Heap!");
  }

  vTaskDelete(NULL); 
}

void loop() {}
