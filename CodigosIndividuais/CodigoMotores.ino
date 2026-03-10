/**
 * ╔═══════════════════════════════════════════════════════════════════════════════╗
 * ║  Aegis Rover - Firmware de Integridade e Proteção v4.7 (RTOS)                 ║
 * ║  Arquitetura: ESP32 | Hardware Oficial: Robocore Vespa (H-Bridge)             ║
 * ║  Recursos: Motores Nativos, Telemetria VBat Nativa, Auto-Reconnect Wi-Fi      ║
 * ╚═══════════════════════════════════════════════════════════════════════════════╝
 */

#include <Arduino.h>
#include <RoboCore_Vespa.h> // Biblioteca Oficial
#include <WiFi.h>
#include <ArduinoOTA.h>     // Biblioteca para atualizações Over-The-Air
#include <WebServer.h>
#include <ESPmDNS.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// =================================================================================
// SEÇÃO 1: OBJETOS DE HARDWARE ROBOCORE VESPA
// =================================================================================
VespaMotors motors;
VespaBattery vbat;
VespaLED led;

// Ajustes de Controle PIM (Escala Vespa: -100 a 100)
#define MAX_PWM_NOMINAL      100   
#define DEADBAND_MIN_PWM     10    // Ignora PWM abaixo de 10% para evitar Stall térmico

// =================================================================================
// SEÇÃO 2: GESTÃO DE REDES (DUAL MODE: AP + STA)
// =================================================================================

// Credenciais da Rede Criada pelo Robô (Plano B)
const char* SSID_AP      = "AegisRover";
const char* PASS_AP      = "AegisRover2026";

// Credenciais do Roteador Local (Plano A)
const char* SSID_STA     = "DLINK DIR-3040";
const char* PASS_STA     = "ClaudioADV2026";

SemaphoreHandle_t g_mutex_drive;
WebServer g_web_server(80);

struct DriveState {
    int pwmAtualL = 0, pwmAtualR = 0; // Velocidade física atual
    int pwmAlvoL = 0, pwmAlvoR = 0;
    float bateriaV = 0.0;
    bool enablePIM = true; // PIM / Anti-Stall (TCS)
} g_drive;

// =================================================================================
// SEÇÃO 3: FRONTEND (INTERFACE DE COMANDO)
// =================================================================================

const char HTML_LANDING[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
  <meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Aegis Rover | Command Center</title>
  <script src="https://cdn.tailwindcss.com"></script>
  <style>
    @import url('https://fonts.googleapis.com/css2?family=Orbitron:wght@700&family=Share+Tech+Mono&display=swap');
    body { background-color: #050914; color: #e2e8f0; font-family: 'Share Tech Mono', monospace; }
    .glass-card { background: rgba(24, 27, 31, 0.6); backdrop-filter: blur(12px); border: 1px solid rgba(50, 116, 217, 0.2); border-radius: 8px; }
    .btn-drive { background: #1e293b; border: 1px solid #334155; transition: 0.2s; user-select: none; }
    .btn-drive:active { background: #3b82f6; transform: scale(0.95); }
    .vbat-alert { color: #ef4444; animation: blink 1s infinite; }
    @keyframes blink { 50% { opacity: 0.5; } }
  </style>
</head>
<body class="min-h-screen flex flex-col items-center justify-center p-4">
  <header class="text-center mb-8">
    <h1 class="text-3xl font-black text-white uppercase tracking-tighter" style="font-family:Orbitron">Aegis Rover Control</h1>
    <p class="text-blue-400 text-xs">UGV TACTICAL INTERFACE v4.7 - VESPA OPTIMIZED</p>
  </header>

  <main class="grid grid-cols-1 md:grid-cols-3 gap-6 w-full max-w-5xl">
    
    <!-- Painel de Telemetria -->
    <div class="glass-card p-6 order-2 md:order-1 flex flex-col justify-between">
      <div>
        <h2 class="text-blue-400 text-sm mb-4 font-bold">TELEMETRY & POWER</h2>
        <div class="space-y-2 text-[11px]">
          <div class="bg-gray-900/50 p-2 border-l-2 border-yellow-500 rounded flex justify-between">
            <span>MAIN BATTERY:</span>
            <span id="vbat" class="font-bold text-yellow-400 text-sm">-- V</span>
          </div>
          <div class="bg-gray-900/50 p-2 border-l-2 border-blue-500 rounded mt-2">
            <p class="text-gray-400">NETWORK LINK</p>
            <p>IP: <span id="ipaddr" class="text-blue-400 font-bold text-sm">Detecting...</span></p>
            <p>RSSI: <span id="rssi" class="text-white">--</span> dBm</p>
          </div>
        </div>
      </div>
      <div class="mt-4 pt-4 border-t border-gray-800 text-xs space-y-1">
        <p>MOTOR L: <span id="valL" class="text-blue-400 font-bold">0</span> %</p>
        <p>MOTOR R: <span id="valR" class="text-blue-400 font-bold">0</span> %</p>
      </div>
    </div>

    <!-- Controle Direcional (Forçado a 100% de potência) -->
    <div class="glass-card p-8 flex flex-col items-center order-1 md:order-2">
      <div class="grid grid-cols-3 gap-3">
        <div></div>
        <button onmousedown="cmd(100,100)" onmouseup="cmd(0,0)" ontouchstart="cmd(100,100)" ontouchend="cmd(0,0)" class="btn-drive w-16 h-16 rounded-lg text-xl">W</button>
        <div></div>
        <button onmousedown="cmd(-100,100)" onmouseup="cmd(0,0)" ontouchstart="cmd(-100,100)" ontouchend="cmd(0,0)" class="btn-drive w-16 h-16 rounded-lg text-xl">A</button>
        <button onmousedown="cmd(-100,-100)" onmouseup="cmd(0,0)" ontouchstart="cmd(-100,-100)" ontouchend="cmd(0,0)" class="btn-drive w-16 h-16 rounded-lg text-xl">S</button>
        <button onmousedown="cmd(100,-100)" onmouseup="cmd(0,0)" ontouchstart="cmd(100,-100)" ontouchend="cmd(0,0)" class="btn-drive w-16 h-16 rounded-lg text-xl">D</button>
      </div>
      <p class="mt-4 text-[10px] text-gray-500 uppercase text-center">VESPA MOTOR API DRIVER</p>
    </div>

    <!-- Modos de Operação (Agora Interativos) -->
    <div class="glass-card p-6 order-3">
       <h2 class="text-blue-400 text-sm mb-4 font-bold">SUBSYSTEMS</h2>
       <div class="flex flex-col gap-2">
          <a href="/monitor" class="bg-blue-600/20 border border-blue-500/40 p-2 rounded text-center text-xs hover:bg-blue-600/40 transition">OPEN INTEGRITY MONITOR</a>
          
          <button id="btn-pim" onclick="toggleSys('pim')" class="text-[10px] p-2 rounded bg-green-900/20 text-green-400 border border-green-500/20 text-left hover:brightness-125 transition">
            PIM / ANTI-STALL: ACTIVE
          </button>
       </div>
    </div>
  </main>

  <script>
    function cmd(l, r) { fetch(`/drive?l=${l}&r=${r}`); }
    function toggleSys(sys) { fetch(`/toggle?sys=${sys}`); }

    document.addEventListener('keydown', (e) => {
      if(e.repeat) return;
      const k = e.key.toLowerCase();
      if(k === 'w') cmd(100, 100);
      if(k === 's') cmd(-100, -100);
      if(k === 'a') cmd(-100, 100);
      if(k === 'd') cmd(100, -100);
    });
    document.addEventListener('keyup', (e) => { cmd(0, 0); });

    setInterval(() => {
      fetch('/data').then(r => r.json()).then(d => {
        document.getElementById('rssi').innerText = d.rssi;
        document.getElementById('valL').innerText = d.pL;
        document.getElementById('valR').innerText = d.pR;
        document.getElementById('ipaddr').innerText = window.location.hostname;
        
        const vbatEl = document.getElementById('vbat');
        vbatEl.innerText = d.vbat.toFixed(2) + " V";
        if (d.vbat < 6.5 && d.vbat > 1.0) vbatEl.classList.add('vbat-alert');
        else vbatEl.classList.remove('vbat-alert');

        // Atualiza a UI das Proteções
        const btnPim = document.getElementById('btn-pim');
        if (d.pim) {
            btnPim.className = "text-[10px] p-2 rounded bg-green-900/20 text-green-400 border border-green-500/20 text-left hover:brightness-125 transition";
            btnPim.innerText = "PIM / ANTI-STALL: ACTIVE";
        } else {
            btnPim.className = "text-[10px] p-2 rounded bg-red-900/20 text-red-400 border border-red-500/20 text-left hover:brightness-125 transition";
            btnPim.innerText = "PIM / ANTI-STALL: DISABLED";
        }
      });
    }, 200);
  </script>
</body>
</html>
)rawliteral";

const char HTML_MONITOR[] PROGMEM = "<html><body style='background:#111; color:#eee; font-family:monospace'><h1>Integrity DB</h1><a href='/'>BACK TO CORE</a></body></html>";

// =================================================================================
// SEÇÃO 4: LÓGICA DE CONTROLE (DRIVE ENGINE - VESPA OPTIMIZED)
// =================================================================================

void vTaskDrive(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for(;;) {
        if (xSemaphoreTake(g_mutex_drive, portMAX_DELAY) == pdTRUE) {
            
            // 1. Telemetria de Bateria
            g_drive.bateriaV = vbat.readVoltage() / 1000.0f;
            
            // Sem rampa: transfere o alvo diretamente para a velocidade atual
            g_drive.pwmAtualL = g_drive.pwmAlvoL;
            g_drive.pwmAtualR = g_drive.pwmAlvoR;

            // 3. Aplicação do Filtro Anti-Stall / TCS
            int pwmL_Final = g_drive.pwmAtualL;
            int pwmR_Final = g_drive.pwmAtualR;
            
            // Verifica se a proteção PIM/Stall está ativada
            if (g_drive.enablePIM) {
                if (abs(pwmL_Final) < DEADBAND_MIN_PWM) pwmL_Final = 0;
                if (abs(pwmR_Final) < DEADBAND_MIN_PWM) pwmR_Final = 0;
            }

            // 4. Escrita Física
            motors.setSpeedLeft(pwmL_Final);
            motors.setSpeedRight(pwmR_Final);

            // 5. Feedback Visual
            if (pwmL_Final != 0 || pwmR_Final != 0) {
                led.on();
            } else {
                led.off();
            }

            xSemaphoreGive(g_mutex_drive);
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
    }
}

// =================================================================================
// SEÇÃO 5: ENDPOINTS API E NETWORK TASK
// =================================================================================

void handleDrive() {
    if (g_web_server.hasArg("l") && g_web_server.hasArg("r")) {
        if (xSemaphoreTake(g_mutex_drive, portMAX_DELAY) == pdTRUE) {
            g_drive.pwmAlvoL = constrain(g_web_server.arg("l").toInt(), -100, 100);
            g_drive.pwmAlvoR = constrain(g_web_server.arg("r").toInt(), -100, 100);
            xSemaphoreGive(g_mutex_drive);
        }
        g_web_server.send(200, "text/plain", "OK");
    }
}

// Endpoint para alternar proteções dinamicamente
void handleToggle() {
    if (g_web_server.hasArg("sys")) {
        String sys = g_web_server.arg("sys");
        if (xSemaphoreTake(g_mutex_drive, portMAX_DELAY) == pdTRUE) {
            if (sys == "pim") g_drive.enablePIM = !g_drive.enablePIM;
            xSemaphoreGive(g_mutex_drive);
        }
        g_web_server.send(200, "text/plain", "OK");
    }
}

void handleData() {
    char jsonBuffer[256];
    snprintf(jsonBuffer, sizeof(jsonBuffer), 
        "{\"pL\":%d,\"pR\":%d,\"rssi\":%d,\"vbat\":%.2f,\"pim\":%d}",
        g_drive.pwmAtualL, g_drive.pwmAtualR, (int)WiFi.RSSI(), g_drive.bateriaV, 
        g_drive.enablePIM);
    g_web_server.send(200, "application/json", jsonBuffer);
}

void vTaskNetwork(void *pvParameters) {
    for(;;) {
        if(WiFi.status() != WL_CONNECTED && WiFi.getMode() != WIFI_AP) {
            WiFi.reconnect();
        }
        g_web_server.handleClient();
        ArduinoOTA.handle(); // <-- Processa as requisições de atualização OTA
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// =================================================================================
// SETUP & BOOT
// =================================================================================

void setup() {
    Serial.begin(115200);
    g_mutex_drive = xSemaphoreCreateMutex();

    Serial.println("\n[BOOT] Iniciando Sistema (Vespa Optimized)...");

    WiFi.mode(WIFI_AP_STA); 
    WiFi.softAP(SSID_AP, PASS_AP);
    Serial.printf("[NET] Modo AP (Robo) Online. Acesse por: 192.168.4.1\n");

    WiFi.begin(SSID_STA, PASS_STA);
    
    int t = 0;
    Serial.print("[NET] Conectando ao roteador D-LINK ");
    while (WiFi.status() != WL_CONNECTED && t < 20) { 
        delay(500); 
        Serial.print("."); 
        t++; 
    }
    
    if(WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[NET] Sucesso! Conectado a rede Principal!\n");
        Serial.printf("[NET] IP ATRIBUÍDO: %s\n", WiFi.localIP().toString().c_str());
        MDNS.begin("aegis-rover");
    } else {
        Serial.println("\n[NET] Falha ao conectar no D-LINK. Use a rede 'AegisRover' (IP: 192.168.4.1).");
    }

    // --- CONFIGURAÇÃO OTA ---
    ArduinoOTA.setHostname("aegis-rover");
    ArduinoOTA.setPassword("AegisRover2026");
    ArduinoOTA.onStart([]() {
        Serial.println("[OTA] Iniciando atualização...");
    });
    ArduinoOTA.onEnd([]() {
        Serial.println("\n[OTA] Atualização concluída!");
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("[OTA] Erro [%u]\n", error);
    });
    ArduinoOTA.begin();
    Serial.println("[OTA] Serviço Over-The-Air ativo e pronto.");
    // ------------------------

    g_web_server.on("/", []() { g_web_server.send_P(200, "text/html", HTML_LANDING); });
    g_web_server.on("/drive", handleDrive);
    g_web_server.on("/toggle", handleToggle); // Nova Rota Registrada
    g_web_server.on("/data", handleData);
    g_web_server.on("/monitor", []() { g_web_server.send_P(200, "text/html", HTML_MONITOR); });
    g_web_server.begin();

    xTaskCreatePinnedToCore(vTaskDrive, "Drive", 8192, NULL, 5, NULL, 1); 
    xTaskCreatePinnedToCore(vTaskNetwork, "Net", 8192, NULL, 3, NULL, 0); 
    
    Serial.println("[BOOT] Aegis Rover Totalmente Operacional.");
}

void loop() { 
    vTaskDelete(NULL); 
}
