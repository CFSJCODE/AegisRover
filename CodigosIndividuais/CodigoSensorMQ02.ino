/*
 * -----------------------------------------------------------------------------
 * Projeto: CFSJ TECH Integrated Network & Automation Node
 * Autor: Cláudio Francisco (CEO & Lead Engineer)
 * Plataforma: ESP32 (Dual Core Architecture) / Compatível ESP32-C6 (FreeRTOS)
 * Versão: 2.1.0 (Green IoT / Power Management Update)
 * Data: 2026-02-22
 * -----------------------------------------------------------------------------
 * Descrição Técnica:
 * Firmware híbrido que implementa uma infraestrutura de rede gerenciável
 * (Station + AP + NTP) concorrentemente com um sistema de automação
 * de atuadores (Relé) controlado via UART e Dashboard Web.
 * * * Atualizações de Arquitetura (Power Management & RTOS):
 * - Implementação de Dynamic Frequency Scaling (DFS) com Datalogs de transição.
 * - Modem Sleep 802.11 (WIFI_PS_MIN_MODEM) ativado.
 * - Task Suspension (FreeRTOS) para a rotina do Relé (0% CPU em inatividade).
 * - Polling adaptativo baseado na demanda da interface Web (AJAX/Fetch).
 * -----------------------------------------------------------------------------
 */

#include <WiFi.h>
#include <WebServer.h>
#include <esp_wifi.h>
#include <time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "esp32-hal-cpu.h" // Para controle dinâmico de Clock (DFS)

// =============================================================================
// 1. DEFINIÇÕES DE HARDWARE E CONSTANTES
// =============================================================================

// GPIO
const uint8_t PINO_RELE = 22;

// Credenciais de Rede (WAN/Station)
const char* SSID_STA     = "DLINK DIR-3040";
const char* PASS_STA     = "ClaudioADV2026";

// Credenciais de Rede (LAN/AP)
const char* SSID_AP      = "ESP32-DevKit-C6";
const char* PASS_AP      = "ClaudioADV2026";

// Configuração de IP Estático (WAN)
IPAddress local_IP(192, 168, 0, 55);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);

// Configuração de IP (LAN/AP)
IPAddress ap_local_IP(192, 168, 4, 1);
IPAddress ap_gateway(192, 168, 4, 1);
IPAddress ap_subnet(255, 255, 255, 0);

// NTP Server
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -10800; // GMT-3
const int   daylightOffset_sec = 0;

// Objetos Globais e RTOS
WebServer server(80);
SemaphoreHandle_t relayMutex; // Proteção para acesso concorrente ao estado
volatile bool relayState = false; // Estado volátil lógico

// Handles das Tarefas (Para IPC e Gestão de Energia)
TaskHandle_t TaskRelayHandle;
TaskHandle_t TaskPowerHandle;

// Variáveis de controle de demanda (Power Management)
volatile unsigned long last_web_request_ms = 0;
const unsigned long DASHBOARD_TIMEOUT_MS = 5000; // 5 segundos sem requisição = ocioso
volatile int current_cpu_freq = 0; // Tracking de estado para Datalogging (evita flood)

// =============================================================================
// 2. IMPLEMENTAÇÃO DE CONTROLE DE HARDWARE (Thread-Safe & IPC)
// =============================================================================

/**
 * @brief Retorna o estado atual do relé de forma segura.
 */
bool getRelayState() {
  bool state = false;
  if (xSemaphoreTake(relayMutex, portMAX_DELAY) == pdTRUE) {
    state = relayState;
    xSemaphoreGive(relayMutex);
  }
  return state;
}

/**
 * @brief Altera o estado do relé atualizando a flag e acordando a Task de hardware.
 * Aplica-se o conceito de Inter-Process Communication (IPC).
 */
void setRelayState(bool state) {
  if (xSemaphoreTake(relayMutex, portMAX_DELAY) == pdTRUE) {
    if (relayState != state) {
        relayState = state;
        Serial.printf("\n[KERNEL_IPC] Alteração de estado detectada (%s). Emitindo sinal de vTaskResume...\n", state ? "LIGAR" : "DESLIGAR");
        
        // Acorda a tarefa dedicada ao Hardware assincronamente
        if (TaskRelayHandle != NULL) {
            vTaskResume(TaskRelayHandle); 
        }
    }
    xSemaphoreGive(relayMutex);
  }
}

// =============================================================================
// 3. UTILITÁRIOS E LÓGICA DE REDE
// =============================================================================

String getInternetTime() {
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    return "Sincronizando NTP...";
  }
  char timeStringBuff[50];
  strftime(timeStringBuff, sizeof(timeStringBuff), "%H:%M:%S - %d/%m/%Y", &timeinfo);
  return String(timeStringBuff);
}

String getConnectedClients() {
  wifi_sta_list_t wifi_sta_list;
  esp_wifi_ap_get_sta_list(&wifi_sta_list);

  String clientList = "";
  if (wifi_sta_list.num > 0) {
    clientList += "<ul class='list-group list-group-flush'>";
    for (int i = 0; i < wifi_sta_list.num; i++) {
      wifi_sta_info_t station = wifi_sta_list.sta[i];
      clientList += "<li class='list-group-item d-flex justify-content-between align-items-center small'>MAC: <span class='font-monospace'>";
      for(int j=0; j<6; j++){
        clientList += String(station.mac[j], HEX);
        if(j<5) clientList += ":";
      }
      clientList += "</span></li>";
    }
    clientList += "</ul>";
  } else {
    clientList = "<div class='p-3 text-muted small'>Nenhum cliente conectado à LAN.</div>";
  }
  return clientList;
}

// =============================================================================
// 4. INTERFACE WEB E API (CORE 1)
// =============================================================================

void handleRoot() {
  // O acesso à raiz também denota atividade do usuário
  last_web_request_ms = millis();

  String html = R"rawliteral(
<!DOCTYPE html>
<html lang='pt-BR'>
<head>
  <meta charset='UTF-8'>
  <meta name='viewport' content='width=device-width, initial-scale=1.0'>
  <title>CFSJ TECH | Control Node</title>
  <link href='https://cdn.jsdelivr.net/npm/bootstrap@5.3.0/dist/css/bootstrap.min.css' rel='stylesheet'>
  <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.0.0/css/all.min.css">
  <style>
    :root { --primary-color: #0d6efd; --bg-color: #f8f9fa; }
    body { background-color: var(--bg-color); font-family: 'Segoe UI', sans-serif; }
    .card { border: none; border-radius: 12px; box-shadow: 0 4px 15px rgba(0,0,0,0.05); transition: transform 0.2s; }
    .card:hover { transform: translateY(-2px); }
    .status-indicator { width: 15px; height: 15px; border-radius: 50%; display: inline-block; margin-right: 8px; }
    .status-on { background-color: #198754; box-shadow: 0 0 8px #198754; }
    .status-off { background-color: #dc3545; }
    .btn-control { padding: 15px; font-weight: bold; text-transform: uppercase; letter-spacing: 1px; }
  </style>
</head>
<body>
  <nav class="navbar navbar-expand-lg navbar-dark bg-primary mb-4">
    <div class="container">
      <a class="navbar-brand fw-bold" href="#"><i class="fas fa-microchip me-2"></i>CFSJ TECH</a>
      <span class="navbar-text text-white small">Engenharia de Sistemas</span>
    </div>
  </nav>

  <div class='container'>
    <div class='row g-4'>
      <div class='col-md-12'>
        <div class='card p-3'>
          <div class='row align-items-center'>
            <div class='col-md-6 border-end'>
              <h6 class='text-muted text-uppercase small mb-1'>Tempo de Sistema (NTP)</h6>
              <h3 id='sysTime' class='fw-light'>Carregando...</h3>
            </div>
            <div class='col-md-6 ps-md-4'>
              <h6 class='text-muted text-uppercase small mb-1'>Uptime</h6>
              <p id='uptime' class='lead mb-0'>--</p>
            </div>
          </div>
        </div>
      </div>

      <div class='col-lg-6'>
        <div class='card h-100'>
          <div class='card-header bg-white fw-bold border-bottom-0 py-3'>
            <i class="fas fa-toggle-on me-2 text-primary"></i>Controle de Atuador (GPIO 22)
          </div>
          <div class='card-body text-center'>
            <div class="mb-4">
              <span class="text-muted d-block mb-2">Estado Atual</span>
              <div id="relayStatusBadge" class="badge bg-secondary fs-5 px-4 py-2">
                Desconhecido
              </div>
            </div>
            <div class="d-grid gap-2 col-10 mx-auto">
              <button onclick="controlRelay(1)" class="btn btn-success btn-control shadow-sm">
                <i class="fas fa-power-off me-2"></i>LIGAR SISTEMA
              </button>
              <button onclick="controlRelay(0)" class="btn btn-outline-danger btn-control">
                <i class="fas fa-stop-circle me-2"></i>DESLIGAR SISTEMA
              </button>
            </div>
          </div>
        </div>
      </div>

      <div class='col-lg-6'>
        <div class='card h-100'>
          <div class='card-header bg-white fw-bold border-bottom-0 py-3'>
            <i class="fas fa-wifi me-2 text-primary"></i>Status de Rede
          </div>
          <div class='card-body'>
            <div class="mb-3">
              <h6 class="fw-bold text-success">Interface WAN (Principal)</h6>
              <ul class="list-unstyled small text-muted">
                <li>IP: )rawliteral" + WiFi.localIP().toString() + R"rawliteral(</li>
                <li>SSID: )rawliteral" + String(SSID_STA) + R"rawliteral(</li>
                <li>Sinal: <span id="rssi">--</span> dBm</li>
              </ul>
            </div>
            <hr>
            <div>
              <h6 class="fw-bold text-primary">Interface LAN (Manutenção)</h6>
              <p class="small text-muted mb-1">AP IP: )rawliteral" + WiFi.softAPIP().toString() + R"rawliteral(</p>
              <div id="clientList">Carregando clientes...</div>
            </div>
          </div>
        </div>
      </div>
    </div>
    
    <footer class='text-center mt-5 text-muted small py-3'>
      &copy; 2026 Cláudio Francisco - Engenharia de Computação PUC Minas
    </footer>
  </div>

  <script>
    function updateDashboard() {
      fetch('/api/status')
        .then(response => response.json())
        .then(data => {
          document.getElementById('sysTime').innerText = data.time;
          document.getElementById('uptime').innerText = data.uptime + "s";
          document.getElementById('rssi').innerText = data.rssi;
          
          const statusBadge = document.getElementById('relayStatusBadge');
          if(data.relay) {
            statusBadge.className = 'badge bg-success fs-5 px-4 py-2';
            statusBadge.innerText = 'ATIVO (ON)';
          } else {
            statusBadge.className = 'badge bg-danger fs-5 px-4 py-2';
            statusBadge.innerText = 'INATIVO (OFF)';
          }
        });
    }

    function controlRelay(state) {
      fetch('/api/control?state=' + state)
        .then(response => {
          if(response.ok) updateDashboard();
        });
    }

    setInterval(updateDashboard, 2000); 
    updateDashboard(); 
  </script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}

// API JSON para Status (AJAX) - Polling do Client
void handleApiStatus() {
  // Marca o timestamp de atividade para a Máquina de Estados de Energia (DFS)
  last_web_request_ms = millis();

  String json = "{";
  json += "\"time\":\"" + getInternetTime() + "\",";
  json += "\"uptime\":" + String(millis()/1000) + ",";
  json += "\"rssi\":" + String(WiFi.RSSI()) + ",";
  json += "\"relay\":" + String(getRelayState() ? "true" : "false");
  json += "}";
  server.send(200, "application/json", json);
}

// API para Controle
void handleApiControl() {
  last_web_request_ms = millis(); // Atividade explícita

  if (server.hasArg("state")) {
    int state = server.arg("state").toInt();
    setRelayState(state == 1);
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Bad Request");
  }
}

// =============================================================================
// 5. TAREFAS MULTICORE E GESTÃO DE ENERGIA (FREERTOS)
// =============================================================================

/**
 * @brief Tarefa de Atuação Física (Relé).
 * Estratégia Green IoT: Mantém 0% de CPU quando não há alteração de estado.
 */
void TaskRelayControlCode(void *pvParameters) {
    (void) pvParameters;
    
    // Inicialização segura do hardware antes do ciclo de suspensão
    digitalWrite(PINO_RELE, getRelayState() ? HIGH : LOW);

    for(;;) {
        // Bloqueia a execução da tarefa no início do loop. Garante 0% de uso de CPU até um evento.
        vTaskSuspend(NULL); 
        
        // --- O CÓDIGO SÓ RETORNA PARA ESTA LINHA QUANDO ACORDADO (IPC/vTaskResume) ---
        Serial.println(F("[KERNEL_SCHED] TaskRelay reativada pelo escalonador (vTaskResume)! Atualizando GPIO do Relé."));

        // Efetua a atuação no hardware
        bool currentState = getRelayState();
        digitalWrite(PINO_RELE, currentState ? HIGH : LOW);
        
        // Finaliza informando a nova suspensão
        Serial.println(F("[KERNEL_SCHED] Atuação de Hardware concluída. Auto-Suspendendo TaskRelay. Consumo de CPU de atuação reduzido a 0%."));
    }
}

/**
 * @brief Tarefa de Monitoramento de Energia e DFS (Dynamic Frequency Scaling).
 * Realiza o Polling adaptativo do processador.
 */
void TaskPowerManagerCode(void *pvParameters) {
    (void) pvParameters;

    for(;;) {
        bool isDashboardActive = (millis() - last_web_request_ms) < DASHBOARD_TIMEOUT_MS;
        bool isSystemStressed = getRelayState(); // Se o relé está ativo, consideramos sistema em carga

        if (isDashboardActive || isSystemStressed) {
            // MODO ALTA PERFORMANCE
            if (current_cpu_freq != 160) {
                setCpuFrequencyMhz(160); // Ajusta o Clock do Core para 160MHz
                current_cpu_freq = 160;
                Serial.println(F("[SYS_POWER] Demanda alta confirmada (Dashboard Web Ativo ou Relé Ligado). Clock: 160MHz (Boost). Amostragem Real-Time: 10Hz."));
            }
            vTaskDelay(pdMS_TO_TICKS(100)); // Polling de alta frequência (100ms)
        } else {
            // MODO LOW POWER (IDLE)
            if (current_cpu_freq != 80) {
                setCpuFrequencyMhz(80); // Reduz o Clock pela metade
                current_cpu_freq = 80;
                Serial.println(F("[SYS_POWER] Inatividade detectada na rede e atuadores. Clock: 80MHz (Low Power). Amostragem Passiva: 2Hz."));
            }
            vTaskDelay(pdMS_TO_TICKS(500)); // Polling de economia de energia (500ms)
        }
    }
}

/**
 * @brief Tarefa dedicada ao I/O Serial.
 */
void TaskSerialControl(void *pvParameters) {
  (void) pvParameters;
  Serial.println(F("[CORE 0] Tarefa Serial Iniciada."));

  for (;;) {
    if (Serial.available() > 0) {
      char cmd = Serial.read();
      if (cmd != '\n' && cmd != '\r') {
        if (cmd == '1') {
          Serial.println(F("[UART] Comando recebido: LIGAR"));
          setRelayState(true);
        } else if (cmd == '0') {
          Serial.println(F("[UART] Comando recebido: DESLIGAR"));
          setRelayState(false);
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50)); // Reduzido o rate para economia adicional
  }
}

// =============================================================================
// 6. SETUP E LOOP PRINCIPAL (CORE 1)
// =============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Configuração GPIO
  pinMode(PINO_RELE, OUTPUT);
  digitalWrite(PINO_RELE, LOW);

  // Inicializa Mutex
  relayMutex = xSemaphoreCreateMutex();

  Serial.println(F("\n--- CFSJ TECH: Inicializando Sistema Híbrido Avançado ---"));

  // 1. Configuração de Rede (WiFi Dual Mode) + Otimização de Modem
  WiFi.mode(WIFI_AP_STA);
  esp_wifi_set_ps(WIFI_PS_MIN_MODEM); // HABILITA O MODEM SLEEP (Economia de RF até 40mA)
  Serial.println(F("[SYS_POWER] Rádio Wi-Fi configurado para WIFI_PS_MIN_MODEM."));
  
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println(F("[ERRO] Falha config IP STA."));
  }
  
  if (!WiFi.softAPConfig(ap_local_IP, ap_gateway, ap_subnet)) {
    Serial.println(F("[ERRO] Falha config IP AP."));
  }

  WiFi.begin(SSID_STA, PASS_STA);
  WiFi.softAP(SSID_AP, PASS_AP);

  Serial.print(F("[WIFI] Conectando..."));
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 15) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  Serial.println("");

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[WAN] Conectado! IP: %s\n", WiFi.localIP().toString().c_str());
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  } else {
    Serial.println(F("[WAN] Falha na conexão. Modo AP isolado ativo."));
  }
  Serial.printf("[LAN] AP Criado. IP: %s\n", WiFi.softAPIP().toString().c_str());

  // 2. Definição de Rotas Web
  server.on("/", handleRoot);
  server.on("/api/status", handleApiStatus);
  server.on("/api/control", handleApiControl);
  server.begin();
  Serial.println(F("[HTTP] Servidor Web Iniciado (Core 1)."));

  // 3. Lançamento das Tarefas no FreeRTOS (Arquitetura Distribuída)
  
  // Tarefa de Escuta Serial (I/O)
  xTaskCreatePinnedToCore(
    TaskSerialControl,   
    "SerialTask",        
    2048,                
    NULL,                
    1,                   
    NULL,                
    0                    // Pinada ao Core 0 (se aplicável ao SoC)
  );

  // Tarefa de Atuação de Hardware (Suspendível)
  xTaskCreate(
    TaskRelayControlCode,
    "RelayActTask",
    2048,
    NULL,
    3,                   // Prioridade Alta (Executa rapidamente e suspende)
    &TaskRelayHandle
  );

  // Tarefa de Gestão de Energia e Monitoramento
  xTaskCreate(
    TaskPowerManagerCode,
    "PowerMngTask",
    2048,
    NULL,
    2,                   // Prioridade Média
    &TaskPowerHandle
  );

  Serial.println(F("[SYSTEM] FreeRTOS Ativo. Gerenciamento de Energia Dinâmico e Tarefas Suspendíveis Inicializados."));
}

void loop() {
  // O Loop principal no Core 1 (Application Core) lida apenas com requisições Web
  server.handleClient();
  vTaskDelay(pdMS_TO_TICKS(10)); // Yield RTOS para evitar bloqueio e economizar CPU
}
