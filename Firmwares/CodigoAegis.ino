/**
 * ╔═══════════════════════════════════════════════════════════════════════════════╗
 * ║  Aegis Rover - Firmware Unificado Autônomo (Vespa + LiDAR + BME688 + MQTT)    ║
 * ╚═══════════════════════════════════════════════════════════════════════════════╝
 */

#include <Arduino.h>
#include <WiFi.h>
#include <RoboCore_Vespa.h> // Motores das rodas e Bateria
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

// =================================================================================
// 1. PINOS E HARDWARE
// =================================================================================
// Pinos do LiDAR (Verifique se a Vespa tem o 11 e 10 livres, senão altere aqui)
#define LIDAR_RX_PIN     16
#define LIDAR_MOTOR_PIN  32 

VespaMotors motors;
VespaBattery vbat;
VespaLED led;


#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME680 bme; 

// =================================================================================
// 2. CONFIGURAÇÕES (MQTT, WiFi, LiDAR)
// =================================================================================
const char* SSID_STA     = "DLINK DIR-3040";
const char* PASS_STA     = "ClaudioADV2026";
const char* MQTT_BROKER  = "broker.emqx.io";
const int   MQTT_PORT    = 1883;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Protocolo LD14P
#define LIDAR_BAUD_RATE  230400 
#define POINT_PER_PACK   12     
#define PKG_HEADER       0x54   
#define PKG_BYTE_2       0x2C   
#define PKG_SIZE         47     
#define MAX_VALID_DISTANCE 6000 

constexpr uint8_t LD_OFFSET_SPEED_L     = 2;
constexpr uint8_t LD_OFFSET_SPEED_H     = 3;
constexpr uint8_t LD_OFFSET_START_ANG_L = 4;
constexpr uint8_t LD_OFFSET_START_ANG_H = 5;
constexpr uint8_t LD_OFFSET_END_ANG_L   = 42;
constexpr uint8_t LD_OFFSET_END_ANG_H   = 43;
constexpr uint8_t LD_OFFSET_DATA_START  = 6;
constexpr uint8_t LD_BYTES_PER_POINT    = 3;
constexpr int32_t LD_MAX_ANGLE_CENT     = 36000;

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

// =================================================================================
// 3. VARIÁVEIS DE ESTADO (MONITOR)
// =================================================================================
bool modoAutonomo = false; // Controlado pelo MQTT
HardwareSerial g_lidar_uart(1); 

struct RobotState {
    uint16_t scan_map[360] = {0};
    SemaphoreHandle_t mutex;
} g_robot;

// =================================================================================
// 4. LÓGICA MQTT E CONEXÃO
// =================================================================================
void callbackMQTT(char* topic, byte* payload, unsigned int length) {
    String mensagem = "";
    for (int i = 0; i < length; i++) {
        mensagem += (char)payload[i];
    }
    
    mensagem.trim(); // Limpa espaços extras ou quebras de linha enviadas pelo mqttBOX
    
    Serial.print("[MQTT] Comando Recebido no topico ");
    Serial.print(topic);
    Serial.print(": '"); 
    Serial.print(mensagem); 
    Serial.println("'");

    if (mensagem == "ON") {
        modoAutonomo = true;
        Serial.println("[Aegis] Autonomia LIGADA");
    } else if (mensagem == "OFF") {
        modoAutonomo = false;
        Serial.println("[Aegis] Autonomia DESLIGADA");
    }
}

void reconectarMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("[MQTT] Tentando conectar... ");
    // O nome do client pode ser qualquer um, aqui usei "AegisRoverClient"
    if (mqttClient.connect("AegisRoverClient")) { 
      Serial.println("Conectado!");
      
      // A LINHA MÁGICA QUE PODE ESTAR FALTANDO:
      mqttClient.subscribe("cfsj/aegis/bme688/comando"); 
      
    } else {
      Serial.print("Falha, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" Tentando de novo em 5s");
      vTaskDelay(pdMS_TO_TICKS(5000));
    }
  }
}
// =================================================================================
// 5. PROCESSAMENTO DO LIDAR
// =================================================================================
static inline uint8_t calculateCRC8(const uint8_t* data, size_t length) {
  uint8_t crc = 0; 
  for (size_t i = 0; i < length; i++) crc = CRC8_LUT[(crc ^ data[i]) & 0xFF];
  return crc;
}

void parseAndProcessLidarBuffer(const uint8_t *buf) {
  uint16_t start_ang_cent = (uint16_t)buf[LD_OFFSET_START_ANG_L] | ((uint16_t)buf[LD_OFFSET_START_ANG_H] << 8);
  uint16_t end_ang_cent   = (uint16_t)buf[LD_OFFSET_END_ANG_L] | ((uint16_t)buf[LD_OFFSET_END_ANG_H] << 8);

  int32_t angle_diff_cent = (int32_t)end_ang_cent - (int32_t)start_ang_cent;
  if (angle_diff_cent < 0) angle_diff_cent += LD_MAX_ANGLE_CENT; 
  int32_t step_cent = angle_diff_cent / (POINT_PER_PACK - 1);

  if (xSemaphoreTake(g_robot.mutex, portMAX_DELAY) == pdTRUE) {
    for (int i = 0; i < POINT_PER_PACK; ++i) {
      int base = LD_OFFSET_DATA_START + (i * LD_BYTES_PER_POINT); 
      uint16_t dist = (uint16_t)buf[base] | ((uint16_t)buf[base + 1] << 8);
      if (dist == 0 || dist > MAX_VALID_DISTANCE) continue;

      int32_t angle_cent = (int32_t)start_ang_cent + (int32_t)step_cent * i;
      angle_cent %= LD_MAX_ANGLE_CENT; 
      if (angle_cent < 0) angle_cent += LD_MAX_ANGLE_CENT;

      uint16_t angle_index = (uint16_t)((angle_cent / 100) % 360);
      g_robot.scan_map[angle_index] = dist; 
    }
    xSemaphoreGive(g_robot.mutex); 
  }
}

// =================================================================================
// 6. TAREFAS FREERTOS
// =================================================================================
void vTaskLidar(void *pvParameters) {
  uint8_t buf[PKG_SIZE] = {0}; 
  uint8_t idx = 0; 
  bool synced = false; 

  for(;;) {
    while (g_lidar_uart.available()) {
      uint8_t b = g_lidar_uart.read();
      if (!synced) {
        if (b == PKG_HEADER) { buf[0] = b; idx = 1; synced = true; }
      } else {
        buf[idx++] = b;
        if (idx == 2 && buf[1] != PKG_BYTE_2) { synced = false; idx = 0; continue; }
        if (idx == PKG_SIZE) {
          if (calculateCRC8(buf, PKG_SIZE - 1) == buf[PKG_SIZE - 1]) parseAndProcessLidarBuffer(buf);
          synced = false; idx = 0;
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void vTaskAutonomiaMotores(void *pvParameters) {
    for(;;) {
        uint16_t dist_frente = MAX_VALID_DISTANCE;

        // Calcula a menor distância nos 30 graus da frente do robô (345° a 15°)
        if (xSemaphoreTake(g_robot.mutex, portMAX_DELAY) == pdTRUE) {
            for(int i=345; i<360; i++) {
                if(g_robot.scan_map[i] > 0 && g_robot.scan_map[i] < dist_frente) dist_frente = g_robot.scan_map[i];
            }
            for(int i=0; i<=15; i++) {
                if(g_robot.scan_map[i] > 0 && g_robot.scan_map[i] < dist_frente) dist_frente = g_robot.scan_map[i];
            }
            xSemaphoreGive(g_robot.mutex);
        }

        // Lógica de Inteligência Autônoma
        if (modoAutonomo) {
            if (dist_frente < 400) {
                // Obstáculo detectado! 
                // Para virar no eixo, vamos girar a roda esquerda para frente (-100) 
                // e a direita para trás (-100).
                motors.setSpeedLeft(-100);
                motors.setSpeedRight(-100);

                // IMPORTANTE: Dá um tempo para ele fisicamente conseguir girar 
                // o corpo antes de ler o LiDAR de novo e tentar ir para frente.
                vTaskDelay(pdMS_TO_TICKS(100)); 
            } else {
                // Caminho livre -> Vai para frente
                motors.setSpeedLeft(-100);
                motors.setSpeedRight(100);
            }
        } else {
            // Robô desligado via MQTT -> Rodas paradas
            motors.stop();
        }
        
        vTaskDelay(pdMS_TO_TICKS(50)); // Atualiza os motores a cada 50ms
    }
}

void vTaskSensoresMQTT(void *pvParameters) {
    if (!bme.begin()) { // Já com o endereço corrigido para evitar o erro do BME!
        Serial.println("Falha no BME688!");
    } else {
        bme.setTemperatureOversampling(BME680_OS_8X);
        bme.setHumidityOversampling(BME680_OS_2X);
        bme.setPressureOversampling(BME680_OS_4X);
        bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
        bme.setGasHeater(320, 150);
    }

    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    mqttClient.setCallback(callbackMQTT);

    uint32_t ultimo_publish = 0;

    for(;;) {
        if (WiFi.status() == WL_CONNECTED) {
            if (!mqttClient.connected()) reconectarMQTT();
            
            // O loop agora roda super rápido, ouvindo seus comandos instantaneamente!
            mqttClient.loop(); 
        }

        // Publica os dados apenas se passaram 2000ms (2 segundos)
        if (millis() - ultimo_publish >= 2000) {
            ultimo_publish = millis();

            if (bme.endReading() && mqttClient.connected()) {
                mqttClient.publish("cfsj/aegis/bme688/temperature", String(bme.temperature).c_str());
                mqttClient.publish("cfsj/aegis/bme688/gas", String(bme.gas_resistance / 1000.0).c_str());
                bme.beginReading(); 
            }

            uint16_t dist_absoluta = MAX_VALID_DISTANCE;
            if (xSemaphoreTake(g_robot.mutex, portMAX_DELAY) == pdTRUE) {
                for (int i = 0; i < 360; i++) {
                    if (g_robot.scan_map[i] > 0 && g_robot.scan_map[i] < dist_absoluta) dist_absoluta = g_robot.scan_map[i];
                }
                xSemaphoreGive(g_robot.mutex);
            }

            if (dist_absoluta != MAX_VALID_DISTANCE && mqttClient.connected()) {
                mqttClient.publish("puc/iot/ld14p/menor_distancia", String(dist_absoluta).c_str());
            }
        }

        // Delay de apenas 20 milissegundos para não travar o RTOS
        vTaskDelay(pdMS_TO_TICKS(20)); 
    }
}

void vTaskDecay(void *pvParameters) {
  for(;;) {
    if (xSemaphoreTake(g_robot.mutex, portMAX_DELAY) == pdTRUE) {
      for (int i = 0; i < 360; i++) {
        if (g_robot.scan_map[i] > 0) {
          g_robot.scan_map[i] = (g_robot.scan_map[i] > 50) ? g_robot.scan_map[i] - 50 : 0;
        }
      }
      xSemaphoreGive(g_robot.mutex);
    }
    vTaskDelay(pdMS_TO_TICKS(150));
  }
}

// =================================================================================
// 7. BOOTLOADER E SETUP
// =================================================================================
void setup() {
    Serial.begin(115200);

    // Motor do LiDAR (Giratória constante)
    pinMode(LIDAR_MOTOR_PIN, OUTPUT);
    analogWrite(LIDAR_MOTOR_PIN, 150); // Liga o motor do laser 
    g_lidar_uart.begin(LIDAR_BAUD_RATE, SERIAL_8N1, LIDAR_RX_PIN, -1);

    // Conexão WiFi Simples
    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID_STA, PASS_STA);
    Serial.print("[NET] Conectando WiFi");
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.println(" Conectado!");

    g_robot.mutex = xSemaphoreCreateMutex();
    
    // Inicia as Tarefas em Paralelo
    if (g_robot.mutex != NULL) {
        xTaskCreate(vTaskLidar, "LiDAR_RX", 4096, NULL, 5, NULL);
        xTaskCreate(vTaskAutonomiaMotores, "Drive_AI", 4096, NULL, 4, NULL);
        xTaskCreate(vTaskSensoresMQTT, "Sensors_MQTT", 8192, NULL, 3, NULL);
        xTaskCreate(vTaskDecay, "Map_Decay", 2048, NULL, 1, NULL);
    }
}

void loop() {
    vTaskDelete(NULL); // Loop fica vazio pois o RTOS gerencia tudo
}