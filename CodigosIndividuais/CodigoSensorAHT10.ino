/**
 * ╔═══════════════════════════════════════════════════════════════════════════════╗
 * ║  Aegis System - Módulo Climático AHT10 v1.8 (FreeRTOS Edition)                ║
 * ║  Arquitetura: ESP32-C6 (RISC-V) | Framework: Arduino Core 3.0.0+              ║
 * ║  Desenvolvedor: Cláudio Francisco (CFJ) | CEO Aegis Rover                     ║
 * ║  Ano: 2026 | Foco: Segurança Patrimonial, Monitoramento e Controle Ambiental  ║
 * ╚═══════════════════════════════════════════════════════════════════════════════╝
 * * FUNDAMENTAÇÃO TEÓRICA - PROJETO AEGIS:
 * O sistema Aegis (Égide) opera como um escudo digital proativo. Este firmware 
 * integra telemetria climática de precisão (AHT10) com uma camada de comunicação 
 * Wi-Fi, garantindo determinismo no processamento I2C via FreeRTOS e 
 * redundância na entrega de dados via WebServer SPA.
 */

/**
 * PINAGEM FÍSICA DO SENSOR AHT10 E ATUADORES
 * ════════════════════════════════════════════════════════════════════════════
 *
 * Conector:
 * │ Dispositivo  │ Função           │ GPIO ESP32-C6 │ Observação             │
 * ├──────────────┼──────────────────┼───────────────┼────────────────────────┤
 * │ AHT10 (VIN)  │ Alimentação      │ 3V3           │ Tensão Lógica 3.3V     │
 * │ AHT10 (GND)  │ Terra (GND)      │ GND           │ Referência de massa    │
 * │ AHT10 (SCL)  │ I2C Clock        │ GPIO 7        │ Roteamento via matriz  │
 * │ AHT10 (SDA)  │ I2C Data         │ GPIO 6        │ Roteamento via matriz  │
 * │ RELÉ (IN)    │ Controle de Fan  │ GPIO 5        │ Aciona cooler 12V      │
 * └──────────────┴──────────────────┴───────────────┴────────────────────────┘
 */

// =================================================================================
// SEÇÃO 1: MAPEAMENTO DE HARDWARE E CONSTANTES TÉCNICAS
// =================================================================================

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <time.h>   // [Robô Sentinela] Adicionado suporte a sincronização NTP

// Bibliotecas Nativas do ESP-IDF para Gestão Aprofundada de Energia
#include <esp_wifi.h>
#include <esp_pm.h> // [Robô Sentinela] Adicionado para escalonamento dinâmico de frequência (DFS)

// Inclusão das bibliotecas do núcleo FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// Mapeamento Físico de Pinos (ESP32-C6)
constexpr uint8_t SDA_PIN = 6;
constexpr uint8_t SCL_PIN = 7;
constexpr uint8_t RGB_LED_PIN = 8;   // Feedback visual de status do sistema nativo do C6
constexpr uint8_t RELAY_FAN_PIN = 5; // Acionamento do Relé para o Cooler/Fan 12V

// [Software Engineering] Magic Numbers do I2C extraídos para Constantes Semânticas
constexpr uint8_t AHT10_CMD_CALIBRATE = 0xE1;
constexpr uint8_t AHT10_CMD_MEASURE   = 0xAC;
constexpr uint8_t AHT10_CMD_SOFTRESET = 0xBA;

// Configurações de Rede Local (Station Mode com IP Estático)
const char* SSID_STA       = "DLINK DIR-3040";
const char* PASSWORD_STA   = "ClaudioADV2026";

// Parametrização TCP/IP (Sub-rede 192.168.0.x)
const IPAddress IP_STA       (192, 168, 0, 55);  
const IPAddress GATEWAY_STA  (192, 168, 0, 1);   
const IPAddress SUBNET_STA   (255, 255, 255, 0); 
const IPAddress DNS_STA      (8, 8, 8, 8);       

// =================================================================================
// SEÇÃO 2: ARQUITETURA DE DADOS E GESTÃO DE ESTADO
// =================================================================================

// [Software Engineering] Máquina de estados usando enum class (Strong Typing C++)
enum class AppContext { HOME, NETWORK, MONITOR };
AppContext g_active_context = AppContext::HOME;

// Instâncias Globais e Drivers
WebServer g_web_server(80);
uint8_t g_aht10_endereco = 0; // Armazena o endereço descoberto no I2C Scanner

// [Software Engineering] Agrupamento de Estado (State Object Pattern). 
// Isso otimiza o cache do processador e permite Deep Copies instantâneos protegidos por Mutex.
struct SystemState {
    float temperatura = 0.0f;
    float umidade = 0.0f;
    uint32_t ultima_leitura_ms = 0;
    uint8_t env_state = 0;        // 0 = Verde, 1 = Amarelo, 2 = Vermelho
    uint8_t fan_mode = 0;         // 0 = Automático, 1 = Forçado ON, 2 = Forçado OFF
    bool fan_is_active = false;   // Status real do hardware
    bool sensor_falha = false;
    uint8_t led_r = 0;
    uint8_t led_g = 0;
    uint8_t led_b = 0;
};

// Instância global única de estado e seu Mutex de proteção
SystemState g_state;
SemaphoreHandle_t g_sensor_mutex;

/**
 * Função utilitária atrelada ao hardware para modificar o LED e o Status simultaneamente.
 * NOTA: Deve ser chamada pelo dono da alteração (Task HW) para evitar concorrência no pino.
 */
void updateLedColor(uint8_t r, uint8_t g, uint8_t b) {
  if (g_state.led_r != r || g_state.led_g != g || g_state.led_b != b) {
    if (xSemaphoreTake(g_sensor_mutex, portMAX_DELAY) == pdTRUE) {
      g_state.led_r = r; 
      g_state.led_g = g; 
      g_state.led_b = b; 
      xSemaphoreGive(g_sensor_mutex);
    }
    neopixelWrite(RGB_LED_PIN, r, g, b); // I/O Físico fora do Mutex
  }
}

// =================================================================================
// SEÇÃO 3: DECLARAÇÕES HTML (FRONTEND SPA VIA PROGMEM)
// =================================================================================

const char HTML_LANDING[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Aegis System | Climate Center</title>
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
    <h1 class="text-4xl md:text-6xl font-black text-white drop-shadow-[0_0_15px_rgba(50,116,217,0.8)]">Aegis System</h1>
    <h2 class="text-xl md:text-2xl text-blue-400 mt-2">Climate Hub AHT10</h2>
    <p class="mt-4 text-slate-400 max-w-lg mx-auto text-sm">Plataforma IoT em Rede (Modo Station via DLINK)</p>
  </header>
  <main class="grid grid-cols-1 md:grid-cols-2 gap-8 w-full max-w-4xl z-10">
    <a href="/net" class="glass-card p-8 flex flex-col items-center justify-center text-center group cursor-pointer decoration-none text-slate-200 hover:text-white">
      <div class="w-16 h-16 rounded-full bg-blue-500/10 border border-blue-500/30 flex items-center justify-center mb-6 group-hover:bg-blue-500/20 group-hover:scale-110 transition-all">
        <svg class="w-8 h-8 text-blue-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
          <path stroke-linecap="round" stroke-linejoin="round" stroke-width="1.5" d="M19.428 15.428a2 2 0 00-1.022-.547l-2.387-.477a6 6 0 00-3.86.517l-.318.158a6 6 0 01-3.86.517L6.05 15.21a2 2 0 00-1.806.547"></path>
        </svg>
      </div>
      <h2 class="text-2xl font-bold mb-2">Network Manager</h2>
      <p class="text-sm text-slate-400">Status do link Wi-Fi L2 e IP</p>
    </a>
    <a href="/monitor" class="glass-card p-8 flex flex-col items-center justify-center text-center group cursor-pointer decoration-none text-slate-200 hover:text-white">
      <div class="w-16 h-16 rounded-full bg-green-500/10 border border-green-500/30 flex items-center justify-center mb-6 group-hover:bg-green-500/20 group-hover:scale-110 transition-all">
        <svg class="w-8 h-8 text-green-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
          <path stroke-linecap="round" stroke-linejoin="round" stroke-width="1.5" d="M12 8v4l3 3"></path>
        </svg>
      </div>
      <h2 class="text-2xl font-bold mb-2">Monitoramento AHT10</h2>
      <p class="text-sm text-slate-400">Dashboard Termo-Higrômetro</p>
    </a>
  </main>
  <footer class="absolute bottom-4 text-slate-500 text-xs text-center w-full">
    &copy; 2026 Aegis System | Firmware v1.8 (RTOS)
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
  <title>Aegis System | Network Manager</title>
  <script src="https://cdn.tailwindcss.com"></script>
  <style>
    body { background-color: #111217; color: #c7d0d9; font-family: ui-sans-serif, system-ui, sans-serif; }
    .g-panel { background-color: #181b1f; border: 1px solid #2c3235; border-radius: 4px; padding: 20px; }
    .g-title { font-size: 0.75rem; text-transform: uppercase; letter-spacing: 0.05em; color: #8e99a4; margin-bottom: 12px; font-weight: 600; }
    .val-text { font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace; color: #fff; }
    .status-dot { display: inline-block; width: 10px; height: 10px; border-radius: 50%; box-shadow: 0 0 10px currentColor; }
    .g-btn { background-color: #22252b; border: 1px solid #2c3235; color: #c7d0d9; transition: all 0.2s; padding: 8px 16px; border-radius: 4px; cursor: pointer; }
    .g-btn:hover { background-color: #2c3235; }
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
          <p class="text-xs text-gray-400 uppercase">I2C AHT10 Addr</p>
          <p class="val-text text-lg text-purple-400" id="sysI2c">--</p>
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
        document.getElementById('sysI2c').innerText = "0x" + data.i2c_addr.toString(16).toUpperCase();
        
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
  <title>Aegis System | AHT10 Telemetry</title>
  <script src="https://cdn.tailwindcss.com"></script>
  <style>
    body { background-color: #111217; color: #c7d0d9; font-family: ui-sans-serif, system-ui, sans-serif; }
    .g-panel { background-color: #181b1f; border: 1px solid #2c3235; border-radius: 4px; padding: 20px; }
    .val-text { font-family: ui-monospace; }
    .status-dot { display: inline-block; width: 10px; height: 10px; border-radius: 50%; box-shadow: 0 0 10px currentColor; }
    .g-btn { background-color: #22252b; border: 1px solid #2c3235; padding: 8px 16px; border-radius: 4px; cursor: pointer; transition: 0.3s; color: #c7d0d9; }
    .g-btn:hover { background-color: #2c3235; }
    .sensor-val { font-size: 5rem; font-weight: bold; line-height: 1; }
    .sensor-unit { font-size: 1.5rem; color: #8e99a4; vertical-align: super; }
  </style>
</head>
<body class="min-h-screen flex flex-col p-4 md:p-6">
  <header class="flex flex-col md:flex-row justify-between items-center mb-6 pb-4 border-b border-gray-700">
    <div class="flex items-center gap-4">
      <a href="/" class="g-btn">&larr; HOME</a>
      <div>
        <h1 class="text-2xl font-bold text-white">AHT10 Environment Sensor</h1>
        <p class="text-xs text-gray-400 mt-1 val-text">Telemetria Climática de Alta Precisão</p>
      </div>
    </div>
    <div class="flex items-center gap-4 mt-4 md:mt-0">
      <div class="flex items-center bg-gray-900 border border-gray-700 rounded px-4 py-2 gap-3">
        <span id="ledIndicator" class="status-dot text-gray-500 bg-gray-500"></span>
        <span id="sysStatusBtn" class="val-text text-sm font-bold">WAITING...</span>
      </div>
    </div>
  </header>
  
  <main class="grid grid-cols-1 md:grid-cols-2 gap-6 flex-grow items-start content-start">
    <div class="g-panel flex flex-col items-center justify-center text-center py-10 border-t-4 border-t-orange-500 relative overflow-hidden">
      <div class="absolute top-4 left-4 text-xs font-bold text-gray-500 uppercase tracking-widest">Temperatura</div>
      <div class="mt-4">
        <span id="tempVal" class="sensor-val val-text text-orange-400">--.-</span>
        <span class="sensor-unit val-text">°C</span>
      </div>
      <p id="tempStatus" class="mt-4 text-sm text-gray-400">Aguardando leitura I2C...</p>
    </div>

    <div class="g-panel flex flex-col items-center justify-center text-center py-10 border-t-4 border-t-blue-500 relative overflow-hidden">
      <div class="absolute top-4 left-4 text-xs font-bold text-gray-500 uppercase tracking-widest">Umidade Relativa</div>
      <div class="mt-4">
        <span id="humVal" class="sensor-val val-text text-blue-400">--.-</span>
        <span class="sensor-unit val-text">%</span>
      </div>
      <p id="humStatus" class="mt-4 text-sm text-gray-400">Aguardando leitura I2C...</p>
    </div>

    <div class="g-panel col-span-1 md:col-span-2 border-t-4 border-t-gray-500 flex flex-col items-center justify-center text-center py-8 relative">
      <div class="absolute top-4 left-4 text-xs font-bold text-gray-500 uppercase tracking-widest mb-2">Índice de Conforto Térmico</div>
      
      <div class="absolute top-4 right-4 flex flex-col items-end gap-2">
        <div class="flex items-center gap-2 bg-gray-900 px-3 py-1 rounded border border-gray-700">
          <span id="fanStatusIcon" class="status-dot bg-gray-500"></span>
          <span id="fanStatusText" class="text-xs font-bold text-gray-400 uppercase tracking-widest">FAN OFF</span>
        </div>
        
        <div class="flex bg-gray-900 border border-gray-700 rounded overflow-hidden shadow-lg mt-1">
          <button onclick="setFanMode(0)" id="btnAuto" class="px-3 py-1 text-xs font-bold transition-colors text-gray-400 hover:bg-gray-800">AUTO</button>
          <button onclick="setFanMode(1)" id="btnOn" class="px-3 py-1 text-xs font-bold transition-colors text-gray-400 hover:bg-gray-800 border-l border-gray-700">ON</button>
          <button onclick="setFanMode(2)" id="btnOff" class="px-3 py-1 text-xs font-bold transition-colors text-gray-400 hover:bg-gray-800 border-l border-gray-700">OFF</button>
        </div>
      </div>

      <div id="comfortStatus" class="val-text text-xl md:text-2xl text-gray-400 font-bold mt-8 md:mt-4">Avaliando ambiente...</div>
    </div>
  </main>

  <script>
    // Função disparada pelos botões para enviar o comando ao ESP32 via GET assíncrono
    function setFanMode(mode) {
      fetch('/set_fan?mode=' + mode)
        .then(() => updateMonitor())
        .catch(err => console.error('Erro ao enviar comando:', err));
    }

    function updateMonitor() {
      fetch('/data?ctx=mon')
        .then(r => r.json())
        .then(data => {
          // Atualiza valores textuais
          const t = parseFloat(data.temp).toFixed(1);
          const h = parseFloat(data.hum).toFixed(1);
          
          document.getElementById('tempVal').innerText = t;
          document.getElementById('humVal').innerText = h;
          document.getElementById('tempStatus').innerText = data.sensor_error ? "FALHA NO SENSOR" : "Leitura Estável";
          document.getElementById('humStatus').innerText = data.sensor_error ? "FALHA NO SENSOR" : "Leitura Estável";
          document.getElementById('comfortStatus').innerText = data.sensor_error ? "INDISPONÍVEL" : data.comfort;

          // HMI: Atualiza Interface Visual do FAN/Exaustor
          const fanIcon = document.getElementById('fanStatusIcon');
          const fanText = document.getElementById('fanStatusText');
          if (data.fan_active) {
              fanIcon.className = "status-dot bg-cyan-400 shadow-[0_0_10px_#22d3ee]";
              fanText.className = "text-xs font-bold text-cyan-400 uppercase tracking-widest";
              fanText.innerText = "FAN ON";
          } else {
              fanIcon.className = "status-dot bg-gray-600";
              fanText.className = "text-xs font-bold text-gray-500 uppercase tracking-widest";
              fanText.innerText = "FAN OFF";
          }

          // HMI: Atualiza o Botão de Modo Ativo
          const btnAuto = document.getElementById('btnAuto');
          const btnOn = document.getElementById('btnOn');
          const btnOff = document.getElementById('btnOff');
          
          // Reset Estilo Padrão
          const defaultClass = "px-3 py-1 text-xs font-bold transition-colors text-gray-400 hover:bg-gray-800";
          btnAuto.className = defaultClass;
          btnOn.className = defaultClass + " border-l border-gray-700";
          btnOff.className = defaultClass + " border-l border-gray-700";

          // Destaca o modo atual selecionado
          if (data.fan_mode === 1) {
              btnOn.className = "px-3 py-1 text-xs font-bold transition-colors bg-cyan-600 text-white shadow-[0_0_10px_#0891b2] border-l border-gray-700";
          } else if (data.fan_mode === 2) {
              btnOff.className = "px-3 py-1 text-xs font-bold transition-colors bg-red-600 text-white shadow-[0_0_10px_#dc2626] border-l border-gray-700";
          } else {
              btnAuto.className = "px-3 py-1 text-xs font-bold transition-colors bg-blue-600 text-white shadow-[0_0_10px_#2563eb]";
          }

          // Dinâmica de Cores (Semáforo do Estado Ambiental)
          const tempEl = document.getElementById('tempVal');
          const humEl = document.getElementById('humVal');
          const comfortEl = document.getElementById('comfortStatus');

          if (data.sensor_error) {
              tempEl.className = "sensor-val val-text text-red-500";
              humEl.className = "sensor-val val-text text-red-500";
              comfortEl.className = "val-text text-xl md:text-2xl font-bold text-red-500";
          } else {
              if (data.env_state === 2) {
                  // Estado Crítico: Texto Vermelho
                  tempEl.className = "sensor-val val-text text-red-500";
                  humEl.className = "sensor-val val-text text-red-500";
                  comfortEl.className = "val-text text-xl md:text-2xl font-bold text-red-500";
              } else if (data.env_state === 1) {
                  // Estado de Alerta (Fora dos padrões): Texto Amarelo
                  tempEl.className = "sensor-val val-text text-yellow-400";
                  humEl.className = "sensor-val val-text text-yellow-400";
                  comfortEl.className = "val-text text-xl md:text-2xl font-bold text-yellow-400";
              } else {
                  // Estado Ideal (Padrão): Cores Normais/Verde
                  tempEl.className = "sensor-val val-text text-orange-400";
                  humEl.className = "sensor-val val-text text-blue-400";
                  comfortEl.className = "val-text text-xl md:text-2xl font-bold text-green-400";
              }
          }

          // Gerencia Estado do LED/Status (Header Button)
          const btnEl = document.getElementById('sysStatusBtn');
          const ledEl = document.getElementById('ledIndicator');
          const r = data.led[0], g = data.led[1], b = data.led[2];
          const colorStr = `rgb(${r},${g},${b})`;
          
          ledEl.style.backgroundColor = colorStr;
          btnEl.style.color = colorStr;
          
          if (data.sensor_error) {
              btnEl.innerText = '🔴 I2C TIMEOUT';
          } else {
              btnEl.innerText = '🟢 SENSOR OK';
          }
        })
        .catch(err => console.error('Monitor error:', err));
    }

    setInterval(updateMonitor, 1000);
    updateMonitor();
  </script>
</body>
</html>
)rawliteral";

// =================================================================================
// SEÇÃO 4: LÓGICA DE INFERÊNCIA AMBIENTAL E GERAÇÃO HTTP
// =================================================================================

/**
 * Função: avaliarConfortoTermico
 * Descrição: Analisa as medições atuais traduzindo diretamente os estados:
 * - IDEAL (Verde)
 * - ALERTA (Amarelo) 
 * - CRÍTICO (Vermelho)
 * Sem acentos para compatibilidade com o Serial Monitor.
 */
const String avaliarConfortoTermico(float temp, float hum) {
  String status = "";

  // 1. Avaliação de Temperatura
  if (temp > 35.0) {
    status = "CRITICO: Temperatura > 35C";
  } else if (temp < 20.0 || temp > 26.0) {
    status = "ALERTA: Temperatura fora do ideal";
  } else {
    status = "IDEAL: Temperatura adequada (20-26C)";
  }

  status += " | ";

  // 2. Avaliação de Umidade (Baseada na diretriz E OU da OMS)
  if (hum < 30.0) {
    status += "CRITICO: Muito Seco (<30%)";
  } else if (hum > 70.0) {
    status += "CRITICO: Risco de Mofo (>70%)";
  } else if (hum >= 30.0 && hum < 40.0) {
    status += "ALERTA: Seco (30-40%)";
  } else if (hum > 60.0 && hum <= 70.0) {
    status += "ALERTA: Pesado (60-70%)";
  } else {
    status += "IDEAL: Umidade adequada (40-60%)";
  }

  return status;
}

/**
 * Endpoint de Comando: /set_fan
 * Recebe comandos da interface Web para forçar o acionamento do relé.
 */
void handleSetFan() {
  if (g_web_server.hasArg("mode")) {
    int mode = g_web_server.arg("mode").toInt();
    if (mode >= 0 && mode <= 2) {
      if (xSemaphoreTake(g_sensor_mutex, portMAX_DELAY) == pdTRUE) {
        g_state.fan_mode = mode;
        xSemaphoreGive(g_sensor_mutex);
        Serial.printf("[HMI Command] Modo do Fan alterado para: %d\n", mode);
      }
    }
  }
  g_web_server.send(200, "text/plain", "OK");
}

/**
 * Endpoint Fallback: handleNotFound
 * Melhora a segurança do sistema garantindo resposta 404 apropriada 
 */
void handleNotFound() {
  g_web_server.send(404, "text/plain", "Aegis System: Route Not Found.");
}

/**
 * Função: handleHttpData
 * Descrição: Gera dinamicamente um payload JSON para o Frontend SPA.
 * [Software Engineering] O uso de uma 'struct' local garante Snapshot instantâneo via C++ Deep Copy.
 */
void handleHttpData() {
  if (g_web_server.hasArg("ctx")) {
    String ctx_arg = g_web_server.arg("ctx");
    if (ctx_arg == "home") g_active_context = AppContext::HOME;
    else if (ctx_arg == "net") g_active_context = AppContext::NETWORK;
    else if (ctx_arg == "mon") g_active_context = AppContext::MONITOR;
  }

  uint32_t ms = millis();
  uint32_t sec = ms / 1000;
  uint32_t h = sec / 3600;
  uint32_t m = (sec % 3600) / 60;
  uint32_t s = sec % 60;
  char timeStr[16];
  snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d", h, m, s);

  // [Software Engineering] DTO (Data Transfer Object) Snapshot
  SystemState localState;

  // Seção Crítica Ultra-Rápida O(1) - Copia a Struct Inteira em um ciclo
  if (xSemaphoreTake(g_sensor_mutex, portMAX_DELAY) == pdTRUE) {
    localState = g_state; 
    xSemaphoreGive(g_sensor_mutex); 
  }

  String analiseConforto = avaliarConfortoTermico(localState.temperatura, localState.umidade);

  // [Software Engineering] Otimização de Strings: Usamos `+=` ao invés de várias alocações locais com `+`
  String json_payload;
  json_payload.reserve(384); 
  
  json_payload += "{";
  json_payload += "\"uptime\":" + String(sec) + ",";
  json_payload += "\"time\":\"" + String(timeStr) + "\",";
  json_payload += "\"rssi\":" + String(WiFi.RSSI()) + ",";
  json_payload += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  json_payload += "\"i2c_addr\":" + String(g_aht10_endereco) + ",";
  
  json_payload += "\"led\":[" + String(localState.led_r) + "," + String(localState.led_g) + "," + String(localState.led_b) + "],";
  json_payload += "\"temp\":" + String(localState.temperatura, 2) + ",";
  json_payload += "\"hum\":" + String(localState.umidade, 2) + ",";
  json_payload += "\"env_state\":" + String(localState.env_state) + ","; 
  json_payload += "\"fan_active\":" + String(localState.fan_is_active ? "true" : "false") + ",";
  json_payload += "\"fan_mode\":" + String(localState.fan_mode) + ",";
  json_payload += "\"comfort\":\"" + analiseConforto + "\",";
  json_payload += "\"sensor_error\":" + String(localState.sensor_falha ? "true" : "false");
  json_payload += "}";
  
  g_web_server.send(200, "application/json", json_payload);
}

// =================================================================================
// SEÇÃO 5: DEFINIÇÃO DAS TAREFAS PREEMPTIVAS DO FREERTOS
// =================================================================================

/**
 * Task 1: Polling I2C do Sensor AHT10 e Regras de Negócio
 * Prioridade: Alta (5) 
 */
void vTaskSensor(void *pvParameters) {
  vTaskDelay(pdMS_TO_TICKS(100)); // Delay para estabilização após Boot
  
  if (g_aht10_endereco != 0) {
      // Soft reset antes da calibração garante um estado limpo
      Wire.beginTransmission(g_aht10_endereco);
      Wire.write(AHT10_CMD_SOFTRESET); 
      Wire.endTransmission();
      vTaskDelay(pdMS_TO_TICKS(20));

      Wire.beginTransmission(g_aht10_endereco);
      Wire.write(AHT10_CMD_CALIBRATE); 
      Wire.write(0x08);
      Wire.write(0x00);
      Wire.endTransmission();
      vTaskDelay(pdMS_TO_TICKS(20)); 
  }

  const TickType_t xFrequency = pdMS_TO_TICKS(2000); // Polling a cada 2 segundos exatos
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for(;;) {
    if (g_aht10_endereco != 0) {
      Wire.beginTransmission(g_aht10_endereco);
      Wire.write(AHT10_CMD_MEASURE);
      Wire.write(0x33);
      Wire.write(0x00);
      
      if (Wire.endTransmission() == 0) {
        vTaskDelay(pdMS_TO_TICKS(80)); // Aguarda conversão

        Wire.requestFrom(g_aht10_endereco, (uint8_t)6);
        if (Wire.available() == 6) {
          uint8_t dados[6];
          for (int i = 0; i < 6; i++) {
            dados[i] = Wire.read();
          }

          uint32_t umidade_raw = ((uint32_t)dados[1] << 12) | ((uint32_t)dados[2] << 4) | (dados[3] >> 4);
          uint32_t temp_raw = (((uint32_t)(dados[3] & 0x0F)) << 16) | ((uint32_t)dados[4] << 8) | dados[5];

          float calc_umidade = ((float)umidade_raw * 100.0) / 1048576.0;
          float calc_temperatura = ((float)temp_raw * 200.0) / 1048576.0 - 50.0;

          uint8_t estado_calc = 0;
          
          if (calc_temperatura > 35.0 || calc_umidade < 30.0 || calc_umidade > 70.0) {
            estado_calc = 2; // Vermelho
          } 
          else if (calc_temperatura < 20.0 || calc_temperatura > 26.0 || (calc_umidade >= 30.0 && calc_umidade < 40.0) || (calc_umidade > 60.0 && calc_umidade <= 70.0)) {
            estado_calc = 1; // Amarelo
          } 
          else {
            estado_calc = 0; // Verde
          }

          String analiseConforto = avaliarConfortoTermico(calc_temperatura, calc_umidade);
          Serial.printf("[Telemetria AHT10] T: %.1f C | U: %.1f %% => %s\n", 
                        calc_temperatura, calc_umidade, analiseConforto.c_str());

          // [Software Engineering] O Mutex agora consolida o estado de uma só vez
          if (xSemaphoreTake(g_sensor_mutex, portMAX_DELAY) == pdTRUE) {
            g_state.temperatura = calc_temperatura;
            g_state.umidade = calc_umidade;
            g_state.env_state = estado_calc; 
            g_state.sensor_falha = false;
            g_state.ultima_leitura_ms = millis();
            xSemaphoreGive(g_sensor_mutex);
          }
        } else {
          if (xSemaphoreTake(g_sensor_mutex, portMAX_DELAY) == pdTRUE) {
            g_state.sensor_falha = true;
            xSemaphoreGive(g_sensor_mutex);
          }
        }
      } else {
        if (xSemaphoreTake(g_sensor_mutex, portMAX_DELAY) == pdTRUE) {
          g_state.sensor_falha = true;
          xSemaphoreGive(g_sensor_mutex);
        }
      }
    } else {
      if (xSemaphoreTake(g_sensor_mutex, portMAX_DELAY) == pdTRUE) {
          g_state.sensor_falha = true;
          xSemaphoreGive(g_sensor_mutex);
      }
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

/**
 * Task 2: Máquina de Estados Visuais e Atuadores (LED e Relé)
 * Prioridade: Normal (4)
 */
void vTaskHardware(void *pvParameters) {
  for(;;) {
    SystemState localState;
    
    // Puxa um snapshot do estado em O(1)
    if (xSemaphoreTake(g_sensor_mutex, portMAX_DELAY) == pdTRUE) {
      localState = g_state;
      xSemaphoreGive(g_sensor_mutex);
    }

    // =================================================================
    // LÓGICA DE CONTROLE DO ATUADOR (RELÉ / FAN) COM OVERRIDE MANUAL
    // =================================================================
    bool fan_deve_ligar = false;

    if (localState.fan_mode == 1) {
        fan_deve_ligar = true; // Override Manual: Forçado ON
    } 
    else if (localState.fan_mode == 2) {
        fan_deve_ligar = false; // Override Manual: Forçado OFF
    } 
    else {
        // Modo Automático (0)
        if (localState.env_state == 2 || localState.env_state == 1) {
            fan_deve_ligar = true; // Liga o exaustor se estiver Amarelo ou Vermelho
        } else {
            fan_deve_ligar = false; // Desliga no Verde
        }
        
        // Proteção extra: se estiver no automático e a rede cair ou o sensor falhar, desliga o Fan
        if (WiFi.status() != WL_CONNECTED || localState.sensor_falha) {
            fan_deve_ligar = false;
        }
    }

    // Aplica o sinal físico ao pino do Relé
    digitalWrite(RELAY_FAN_PIN, fan_deve_ligar ? HIGH : LOW);

    // Sincroniza o estado real do pino para o Frontend Web
    if (localState.fan_is_active != fan_deve_ligar) {
      if (xSemaphoreTake(g_sensor_mutex, portMAX_DELAY) == pdTRUE) { 
          g_state.fan_is_active = fan_deve_ligar; 
          xSemaphoreGive(g_sensor_mutex); 
      }
    }

    // =================================================================
    // LÓGICA DE CONTROLE VISUAL (LED RGB DA PLACA)
    // =================================================================
    if (WiFi.status() != WL_CONNECTED) {
      updateLedColor(0, 0, 255); // Azul Fixo quando cai a rede
    } else if (localState.sensor_falha) {
      updateLedColor(255, 128, 0); // Laranja para falha de I2C
    } else {
      if (localState.env_state == 2) updateLedColor(255, 0, 0);       // Vermelho (Crítico)
      else if (localState.env_state == 1) updateLedColor(255, 255, 0); // Amarelo (Alerta)
      else updateLedColor(0, 255, 0);                                 // Verde (Ideal)
    }
    
    vTaskDelay(pdMS_TO_TICKS(250)); // Avalia e atualiza as GPIOs a 4Hz
  }
}

/**
 * Task 3: Camada LwIP e Servidor Web HTTP
 * Prioridade: Normal (3)
 */
void vTaskNetwork(void *pvParameters) {
  // [Robô Sentinela] Timer base para o Watchdog de Conectividade
  uint32_t last_wifi_check = millis();

  for(;;) {
    g_web_server.handleClient(); // Processa requests HTTP de forma não-bloqueante
    
    // [Robô Sentinela] Watchdog de Conectividade: Operação Remota Autônoma
    // Vital para plataformas móveis: Se o link principal cair por > 60s, força reinício da base de RF.
    if (WiFi.status() != WL_CONNECTED) {
        if (millis() - last_wifi_check > 60000) {
            Serial.println("[Watchdog] Falha critica de rede (Link RF). Reiniciando o sentinela para auto-recovery...");
            ESP.restart();
        }
    } else {
        last_wifi_check = millis(); // Alimentando o Watchdog de conectividade
    }

    vTaskDelay(pdMS_TO_TICKS(10)); // Cede tempo para não monopolizar a fila RTOS
  }
}

// =================================================================================
// SEÇÃO 6: BOOTLOADER E ESCALONADOR RTOS (SETUP)
// =================================================================================

void setup() {
  Serial.begin(115200); 
  
  uint32_t t = millis();
  while (!Serial && (millis() - t < 3000)) { delay(10); }
  
  Serial.println("\n===============================================================================");
  Serial.println("  Aegis System - AHT10 Sensor Hub v1.8 (FreeRTOS)");
  Serial.println("  Arquitetura: ESP32-C6 (RISC-V) | Framework: Arduino Core 3.0.0+");
  Serial.println("===============================================================================\n");
  
  // Inicialização de GPIOs Visuais e Físicos
  pinMode(RGB_LED_PIN, OUTPUT); 
  updateLedColor(0, 0, 255); // Azul = Inicializando
  
  pinMode(RELAY_FAN_PIN, OUTPUT);
  digitalWrite(RELAY_FAN_PIN, LOW); // Garante que o relé inicie desligado

  // 1. INICIALIZAÇÃO E SCAN I2C
  Serial.println("[Hardware] Inicializando barramento I2C...");
  Wire.begin(SDA_PIN, SCL_PIN);
  
  for (byte endereco = 1; endereco < 127; endereco++) {
    Wire.beginTransmission(endereco);
    if (Wire.endTransmission() == 0) {
      if (g_aht10_endereco == 0) {
        g_aht10_endereco = endereco; 
        Serial.printf("[Hardware] Sensor AHT10 detectado no endereço: 0x%X\n", endereco);
      }
    }
  }

  if (g_aht10_endereco == 0) {
    Serial.println("[Hardware] ERRO: Nenhum dispositivo I2C encontrado!");
  }

  // 2. CONFIGURAÇÃO DE REDE WI-FI
  Serial.println("[Network] Tentando conexao com a Infraestrutura...");
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);

  if (!WiFi.config(IP_STA, GATEWAY_STA, SUBNET_STA, DNS_STA)) {
      Serial.println("[Network] AVISO: Falha ao aplicar IP Estatico.");
  }
  WiFi.begin(SSID_STA, PASSWORD_STA);
  
  // [Robô Sentinela] Green IoT: Otimização energética e gestão da bateria via DFS (Dynamic Frequency Scaling)
  esp_wifi_set_ps(WIFI_PS_MIN_MODEM); // Power Saving Profile base
  
  #if CONFIG_PM_ENABLE
  esp_pm_config_t pm_config = {
      .max_freq_mhz = 160,
      .min_freq_mhz = 40,   // Redução drástica de clock durante idle (esperando inputs de sensores/HTTP)
      .light_sleep_enable = true
  };
  esp_pm_configure(&pm_config);
  Serial.println("[Aegis System] Power Management / DFS habilitado para autonomia máxima.");
  #endif
  
  // [Robô Sentinela] Configuração de NTP para sincronização de relógio global
  configTime(-10800, 0, "pool.ntp.org", "time.nist.gov");
  Serial.println("[Aegis System] NTP Configurado (Fuso: -3h).");
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) { 
    delay(500); 
    Serial.print("."); 
    attempts++; 
  }
  
  if (WiFi.status() == WL_CONNECTED) { 
      Serial.println("\n[Network] Conectado! IP: " + WiFi.localIP().toString());
      MDNS.begin("aegis-climate");
  } else {
      Serial.println("\n[Network] Falha na conexao. Modo Autônomo ativo. Retentativas ocorrendo em background.");
  }

  // 3. DECLARAÇÃO DOS ENDPOINTS WEB
  g_web_server.on("/", []() { g_web_server.send_P(200, "text/html", HTML_LANDING); });
  g_web_server.on("/net", []() { g_web_server.send_P(200, "text/html", HTML_NETWORK); });
  g_web_server.on("/monitor", []() { g_web_server.send_P(200, "text/html", HTML_MONITOR); });
  g_web_server.on("/set_fan", handleSetFan);
  g_web_server.on("/data", handleHttpData);
  g_web_server.onNotFound(handleNotFound);
  g_web_server.begin();
  Serial.println("[Server] WebServer SPA inicializado.");

  // 4. ALOCAÇÃO DE TAREFAS (RTOS Scheduler)
  Serial.println("[Aegis System] Instanciando Mutexes e RTOS Tasks...");
  Serial.println("[Aegis System] Iniciando controle e monitoramento ambiental continuo...\n");
  
  g_sensor_mutex = xSemaphoreCreateMutex();
  
  if (g_sensor_mutex != NULL) {
    xTaskCreate(vTaskSensor,   "Task_AHT10", 4096, NULL, 5, NULL);
    xTaskCreate(vTaskHardware, "Task_HW",    2048, NULL, 4, NULL); 
    xTaskCreate(vTaskNetwork,  "Task_Net",   4096, NULL, 3, NULL); 
    
    Serial.println("[Aegis System] RTOS Operante. Handover para o Scheduler.");
    Serial.println("-------------------------------------------------------------------------------");
  } else {
    Serial.println("[ERRO CRITICO] Falha ao alocar Mutex na Heap!");
  }

  vTaskDelete(NULL); 
}

void loop() {
  // O loop fica vazio e nunca será atingido devido ao vTaskDelete() no setup.
}
