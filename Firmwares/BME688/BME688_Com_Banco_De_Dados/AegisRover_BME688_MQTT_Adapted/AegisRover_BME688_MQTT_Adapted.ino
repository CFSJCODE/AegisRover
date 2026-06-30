/*
  ============================================================================
  Aegis Rover LD14P + BME688 + MQTT
  ============================================================================
  Plataforma:
    - ESP32 / RoboCore Vespa
    - LiDAR LD14P
    - Sensor ambiental BME688
    - MQTT via PubSubClient
    - OTA via ArduinoOTA

  Objetivo:
    Ler telemetria ambiental do BME688 e publicar em tópicos MQTT compatíveis
    com o cliente Python bd_bme688_mqtt.py, mantendo a leitura do LiDAR LD14P.

  ============================================================================
  PINAGEM DO LD14P
  ============================================================================
  Ligações físicas esperadas:

    LD14P BRANCO   -> RX2 / D16
    LD14P VERMELHO -> S1 / D26 / PWM lógico do motor
    LD14P VERDE    -> GND
    LD14P PRETO    -> 5V

  Observações:
    - O ESP32 apenas recebe dados do LiDAR.
    - TX do ESP32 não é utilizado para o LD14P.
    - O fio vermelho controla o motor do LD14P via PWM lógico.

  ============================================================================
  TÓPICOS MQTT
  ============================================================================
  Broker:
    test.mosquitto.org:1883

  Publicações ESP32 -> MQTT:

    puc/iot/bme688/temperatura
    puc/iot/bme688/pressao
    puc/iot/bme688/umidade
    puc/iot/bme688/gas
    puc/iot/bme688/altitude

  Comandos MQTT -> ESP32:

    puc/iot/bme688/comando

  Comandos reconhecidos:

    delete
    termina

  ============================================================================
  CORREÇÕES ESTRUTURAIS APLICADAS
  ============================================================================
  - Corrigido conflito com NetworkManager interno do ESP32 Arduino Core 3.x.
  - Corrigida chamada não-const de PubSubClient::connected().
  - Corrigida ordem de declaração da classe LidarMotorPwm.
  - Corrigido PWM do LiDAR para GPIO 26.
  - Corrigido loop principal para não reinicializar Wi-Fi continuamente.
*/

#include <Arduino.h>
#include <HardwareSerial.h>
#include <RoboCore_Vespa.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <math.h>

#ifndef ARDUINO_ARCH_ESP32
  #error Este firmware e destinado a ESP32 / RoboCore Vespa.
#endif

// ============================================================================
// CONFIGURAÇÃO CENTRALIZADA DO SISTEMA
// ============================================================================
namespace Config {
  // --------------------------------------------------------------------------
  // Serial de depuração
  // --------------------------------------------------------------------------
  static constexpr uint32_t SERIAL_BAUD = 115200;

  // --------------------------------------------------------------------------
  // Wi-Fi
  // --------------------------------------------------------------------------
  /*
    Caso não use Preferences para armazenar SSID/senha, preencha abaixo.

    Exemplo:
      static const char* WIFI_SSID = "MinhaRede";
      static const char* WIFI_PASS = "MinhaSenha";
  */
  static const char* WIFI_SSID = "MEGANET";
  static const char* WIFI_PASS = "ClaudioJ2006";

  static constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 12000;
  static constexpr uint32_t WIFI_RECONNECT_INTERVAL_MS = 15000;

  // --------------------------------------------------------------------------
  // LiDAR LD14P
  // --------------------------------------------------------------------------
  /*
    Pinagem configurada conforme sua ligação física:

      LD14P BRANCO   -> RX2 / D16
      LD14P VERMELHO -> S1 / D26 / PWM lógico
      LD14P VERDE    -> GND
      LD14P PRETO    -> 5V
  */
  static constexpr uint8_t LIDAR_RX_PIN = 16;
  static constexpr int8_t  LIDAR_TX_PIN = -1;
  static constexpr uint8_t LIDAR_MOTOR_PWM_PIN = 26;

  static constexpr uint32_t LIDAR_BAUD = 230400;

  static constexpr uint32_t LIDAR_PWM_FREQ_HZ = 1000;
  static constexpr uint8_t  LIDAR_PWM_RES_BITS = 10;
  static constexpr uint16_t LIDAR_PWM_MAX = (1U << LIDAR_PWM_RES_BITS) - 1U;
  static constexpr uint16_t LIDAR_PWM_DEFAULT = 512;
  static constexpr uint8_t  LIDAR_PWM_LEDC_CHANNEL = 8;

  // --------------------------------------------------------------------------
  // Protocolo LD14P
  // --------------------------------------------------------------------------
  static constexpr uint8_t LD_SOF = 0x54;
  static constexpr uint8_t LD_FRAME = 0x2C;
  static constexpr uint8_t LD_PACKET_SIZE = 47;
  static constexpr uint8_t LD_POINTS_PER_PACKET = 12;
  static constexpr uint8_t LD_CRC_POLY = 0x4D;
  static constexpr uint16_t ANGLE_MAX_CDEG = 36000;

  // --------------------------------------------------------------------------
  // Parâmetros de leitura do LiDAR
  // --------------------------------------------------------------------------
  static constexpr uint16_t SCAN_MAX_POINTS = 720;
  static constexpr uint16_t MIN_VALID_MM = 100;
  static constexpr uint16_t MAX_VALID_MM = 8000;
  static constexpr uint8_t MIN_INTENSITY = 8;

  // --------------------------------------------------------------------------
  // Grade de ocupação
  // Mantida para expansão futura de navegação/autonomia.
  // --------------------------------------------------------------------------
  static constexpr int16_t GRID_SIZE = 101;
  static constexpr int16_t GRID_CENTER = GRID_SIZE / 2;
  static constexpr int16_t GRID_CELL_MM = 80;

  static constexpr uint8_t CELL_FREE = 0;
  static constexpr uint8_t CELL_UNKNOWN = 90;
  static constexpr uint8_t CELL_SOFT_INFLATED = 130;
  static constexpr uint8_t CELL_LETHAL = 225;
  static constexpr uint8_t CELL_OCCUPIED = 255;

  // --------------------------------------------------------------------------
  // Geometria do robô
  // --------------------------------------------------------------------------
  static constexpr int16_t ROBOT_RADIUS_MM = 135;
  static constexpr int16_t SAFETY_MARGIN_MM = 140;
  static constexpr int16_t OBSTACLE_RANGE_MM = 2500;

  // --------------------------------------------------------------------------
  // Intervalos de controle
  // --------------------------------------------------------------------------
  static constexpr uint32_t REPLAN_INTERVAL_MS = 180;
  static constexpr uint32_t CONTROL_INTERVAL_MS = 30;
  static constexpr uint32_t TELEMETRY_INTERVAL_MS = 1000;
  static constexpr uint32_t NO_SCAN_TIMEOUT_MS = 1200;

  // --------------------------------------------------------------------------
  // Intervalos MQTT
  // --------------------------------------------------------------------------
  static constexpr uint32_t MQTT_PUBLISH_INTERVAL_MS = 5000;
  static constexpr uint32_t MQTT_RECONNECT_DELAY_MS = 2500;

  // --------------------------------------------------------------------------
  // Parâmetros de navegação
  // Mantidos para expansão futura.
  // --------------------------------------------------------------------------
  static constexpr int16_t DEFAULT_GOAL_X_MM = 3000;
  static constexpr int16_t DEFAULT_GOAL_Y_MM = 0;
  static constexpr int16_t GOAL_REACHED_MM = 230;
  static constexpr int16_t FRONT_STOP_MM = 330;
  static constexpr int16_t FRONT_SLOW_MM = 850;

  static constexpr float MAX_LINEAR_PCT = 56.0f;
  static constexpr float MAX_ANGULAR_PCT = 68.0f;

  // --------------------------------------------------------------------------
  // OTA
  // --------------------------------------------------------------------------
  static const char* OTA_HOSTNAME = "AegisRover-LD14P";
  static const char* OTA_PASS = "AegisOTA";

  // --------------------------------------------------------------------------
  // MQTT
  // --------------------------------------------------------------------------
  static const char* MQTT_BROKER = "test.mosquitto.org";
  static constexpr uint16_t MQTT_PORT = 1883;
  static const char* MQTT_CLIENT_ID = "ESP32_BME688_AegisRover";

  static const char* MQTT_TOPIC_PREFIX = "puc/iot/bme688";

  static const char* MQTT_TOPIC_TEMP = "puc/iot/bme688/temperatura";
  static const char* MQTT_TOPIC_PRES = "puc/iot/bme688/pressao";
  static const char* MQTT_TOPIC_HUM  = "puc/iot/bme688/umidade";
  static const char* MQTT_TOPIC_GAS  = "puc/iot/bme688/gas";
  static const char* MQTT_TOPIC_ALT  = "puc/iot/bme688/altitude";
  static const char* MQTT_TOPIC_CMD  = "puc/iot/bme688/comando";

  // --------------------------------------------------------------------------
  // BME688
  // --------------------------------------------------------------------------
  static constexpr uint32_t BME688_READ_INTERVAL_MS = 5000;
  static constexpr float SEA_LEVEL_PRESSURE_HPA = 1013.25f;

  static constexpr uint8_t BME688_ADDR_PRIMARY = 0x77;
  static constexpr uint8_t BME688_ADDR_SECONDARY = 0x76;
}

// ============================================================================
// UTILITÁRIOS MATEMÁTICOS
// ============================================================================
namespace Util {
  static constexpr float PI_F = 3.14159265358979323846f;
  static constexpr float DEG_PER_RAD = 57.29577951308232f;
  static constexpr float RAD_PER_CDEG = PI_F / 18000.0f;

  /**
   * @brief Limita um valor float dentro de um intervalo fechado.
   */
  float clampFloat(float value, float low, float high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
  }

  /**
   * @brief Limita um valor inteiro dentro de um intervalo fechado.
   */
  int clampInt(int value, int low, int high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
  }

  /**
   * @brief Normaliza ângulo em radianos para o intervalo [-pi, pi).
   */
  float normalizeRad(float rad) {
    while (rad >= PI_F) rad -= 2.0f * PI_F;
    while (rad < -PI_F) rad += 2.0f * PI_F;
    return rad;
  }
}

// ============================================================================
// ESTRUTURAS DE DADOS
// ============================================================================

/**
 * @brief Ponto polar individual do LiDAR.
 */
struct LidarPoint {
  uint16_t angleCdeg = 0;     // Ângulo em centésimos de grau.
  uint16_t distanceMm = 0;    // Distância em milímetros.
  uint8_t intensity = 0;      // Intensidade/refletância.
};

/**
 * @brief Varredura completa aproximada do LiDAR.
 */
struct LidarScan {
  LidarPoint points[Config::SCAN_MAX_POINTS];
  uint16_t count = 0;
  uint16_t speedDps = 0;
  uint32_t completedAtMs = 0;
};

/**
 * @brief Sumário setorial para futura lógica de navegação.
 */
struct SectorSummary {
  uint16_t front = 0;
  uint16_t left = 0;
  uint16_t right = 0;
  uint16_t nearest = 0;
};

/**
 * @brief Leitura ambiental consolidada do BME688.
 */
struct Bme688Reading {
  float temperatureC = 0.0f;
  float humidityPct = 0.0f;
  float pressureHpa = 0.0f;
  float gasKohm = 0.0f;
  float altitudeM = 0.0f;
  uint32_t completedAtMs = 0;
};

// ============================================================================
// CONTROLE PWM DO MOTOR DO LD14P
// ============================================================================

/**
 * @brief Controla o sinal PWM lógico do motor do LiDAR LD14P.
 *
 * No ESP32 Arduino Core 3.x, usa ledcAttach(pin, freq, resolution).
 * No Core 2.x, usa ledcSetup + ledcAttachPin.
 */
class LidarMotorPwm {
private:
  uint16_t duty = Config::LIDAR_PWM_DEFAULT;

public:
  /**
   * @brief Inicializa o periférico LEDC para o PWM do motor do LiDAR.
   */
  void begin() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    const bool attached = ledcAttach(
      Config::LIDAR_MOTOR_PWM_PIN,
      Config::LIDAR_PWM_FREQ_HZ,
      Config::LIDAR_PWM_RES_BITS
    );

    if (!attached) {
      Serial.println("[LD14P-PWM] Falha ao anexar LEDC ao pino PWM.");
      return;
    }

    ledcWrite(Config::LIDAR_MOTOR_PWM_PIN, duty);
#else
    ledcSetup(
      Config::LIDAR_PWM_LEDC_CHANNEL,
      Config::LIDAR_PWM_FREQ_HZ,
      Config::LIDAR_PWM_RES_BITS
    );

    ledcAttachPin(Config::LIDAR_MOTOR_PWM_PIN, Config::LIDAR_PWM_LEDC_CHANNEL);
    ledcWrite(Config::LIDAR_PWM_LEDC_CHANNEL, duty);
#endif

    Serial.printf(
      "[LD14P-PWM] PWM iniciado no GPIO %u com duty %u/%u.\n",
      Config::LIDAR_MOTOR_PWM_PIN,
      duty,
      Config::LIDAR_PWM_MAX
    );
  }

  /**
   * @brief Ajusta o duty cycle do PWM do motor.
   */
  void setDuty(uint16_t newDuty) {
    duty = static_cast<uint16_t>(
      Util::clampInt(newDuty, 0, Config::LIDAR_PWM_MAX)
    );

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(Config::LIDAR_MOTOR_PWM_PIN, duty);
#else
    ledcWrite(Config::LIDAR_PWM_LEDC_CHANNEL, duty);
#endif
  }

  /**
   * @brief Retorna o duty cycle atualmente configurado.
   */
  uint16_t getDuty() const {
    return duty;
  }
};

// ============================================================================
// GERENCIADOR DO SENSOR BME688
// ============================================================================

/**
 * @brief Gerencia inicialização, configuração e leitura não bloqueante do BME688.
 */
class Bme688Manager {
private:
  Adafruit_BME680 bme;
  Bme688Reading reading;

  bool available = false;
  bool readingActive = false;

  uint32_t readyAtMs = 0;
  uint32_t lastStartMs = 0;

  uint8_t activeAddress = 0;

  /**
   * @brief Configura oversampling, filtro IIR e aquecedor de gás.
   */
  void configureSensor() {
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150);
  }

public:
  /**
   * @brief Inicializa o barramento I2C e detecta o BME688 em 0x77 ou 0x76.
   */
  void begin() {
    Wire.begin();

    if (bme.begin(Config::BME688_ADDR_PRIMARY)) {
      activeAddress = Config::BME688_ADDR_PRIMARY;
      available = true;
    } else if (bme.begin(Config::BME688_ADDR_SECONDARY)) {
      activeAddress = Config::BME688_ADDR_SECONDARY;
      available = true;
    }

    if (!available) {
      Serial.println("[BME688] Sensor nao encontrado em 0x77 ou 0x76.");
      return;
    }

    configureSensor();

    // Força a primeira leitura assim que o loop iniciar.
    lastStartMs = millis() - Config::BME688_READ_INTERVAL_MS;

    Serial.printf("[BME688] Online em I2C 0x%02X.\n", activeAddress);
  }

  /**
   * @brief Executa ciclo de leitura não bloqueante.
   *
   * A biblioteca Adafruit permite iniciar a leitura e finalizar depois
   * do tempo necessário de medição, evitando bloquear o loop principal.
   */
  void update() {
    if (!available) return;

    const uint32_t now = millis();

    if (!readingActive && (now - lastStartMs) >= Config::BME688_READ_INTERVAL_MS) {
      const unsigned long endTime = bme.beginReading();

      if (endTime == 0) {
        lastStartMs = now;
        Serial.println("[BME688] Falha ao iniciar leitura.");
        return;
      }

      readingActive = true;
      readyAtMs = static_cast<uint32_t>(endTime);
      lastStartMs = now;
      return;
    }

    if (readingActive && static_cast<int32_t>(now - readyAtMs) >= 0) {
      readingActive = false;

      if (!bme.endReading()) {
        Serial.println("[BME688] Falha ao finalizar leitura.");
        return;
      }

      reading.temperatureC = bme.temperature;
      reading.humidityPct = bme.humidity;
      reading.pressureHpa = bme.pressure / 100.0f;
      reading.gasKohm = bme.gas_resistance / 1000.0f;
      reading.altitudeM = bme.readAltitude(Config::SEA_LEVEL_PRESSURE_HPA);
      reading.completedAtMs = now;
    }
  }

  bool isAvailable() const {
    return available;
  }

  bool hasReading() const {
    return available && reading.completedAtMs > 0;
  }

  const Bme688Reading& latest() const {
    return reading;
  }
};

// ============================================================================
// GERENCIADOR MQTT
// ============================================================================

/**
 * @brief Gerencia conexão MQTT, assinatura de comandos e publicação BME688.
 */
class MqttManager {
private:
  WiFiClient wifiClient;
  PubSubClient client;

  uint32_t lastReconnectMs = 0;
  bool connectedFlag = false;

  /**
   * @brief Callback de recebimento MQTT.
   *
   * Recebe comandos do tópico:
   *   puc/iot/bme688/comando
   */
  void onMessageReceived(char* topic, byte* payload, unsigned int length) {
    if (length == 0) return;

    char command[128];

    const unsigned int copyLen =
      length < (sizeof(command) - 1) ? length : (sizeof(command) - 1);

    memcpy(command, payload, copyLen);
    command[copyLen] = '\0';

    Serial.printf("[MQTT-RX] %s = %s\n", topic, command);

    String cmd(command);
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "DELETE") {
      Serial.println("[CMD] DELETE recebido. Tratamento principal fica no cliente Python.");
    } else if (cmd == "TERMINA") {
      Serial.println("[CMD] TERMINA recebido. Cliente Python deve encerrar.");
    } else {
      Serial.println("[CMD] Comando MQTT desconhecido.");
    }
  }

  /**
   * @brief Tenta estabelecer conexão com o broker MQTT.
   */
  bool connectToBroker() {
    Serial.printf(
      "[MQTT] Conectando a %s:%u...\n",
      Config::MQTT_BROKER,
      Config::MQTT_PORT
    );

    String clientId = String(Config::MQTT_CLIENT_ID) + "_" + WiFi.macAddress();
    clientId.replace(":", "");

    if (client.connect(clientId.c_str())) {
      connectedFlag = true;

      Serial.println("[MQTT] Conectado.");

      const bool subscribed = client.subscribe(Config::MQTT_TOPIC_CMD);

      if (subscribed) {
        Serial.printf("[MQTT] Inscrito em: %s\n", Config::MQTT_TOPIC_CMD);
      } else {
        Serial.printf("[MQTT] Falha ao assinar: %s\n", Config::MQTT_TOPIC_CMD);
      }

      return true;
    }

    connectedFlag = false;

    Serial.printf("[MQTT] Falha. rc=%d\n", client.state());
    return false;
  }

public:
  MqttManager() : client(wifiClient) {}

  /**
   * @brief Configura servidor MQTT e callback.
   */
  void begin() {
    client.setServer(Config::MQTT_BROKER, Config::MQTT_PORT);

    client.setCallback([this](char* topic, byte* payload, unsigned int length) {
      this->onMessageReceived(topic, payload, length);
    });
  }

  /**
   * @brief Mantém MQTT vivo e tenta reconectar sem bloquear o loop.
   */
  void update() {
    if (WiFi.status() != WL_CONNECTED) {
      connectedFlag = false;
      return;
    }

    if (!client.connected()) {
      connectedFlag = false;

      const uint32_t now = millis();

      if ((now - lastReconnectMs) >= Config::MQTT_RECONNECT_DELAY_MS) {
        lastReconnectMs = now;
        connectToBroker();
      }

      return;
    }

    connectedFlag = true;
    client.loop();
  }

  /**
   * @brief Retorna estado de conexão MQTT.
   *
   * Importante:
   * PubSubClient::connected() não é const. Por isso este método também
   * não deve ser const.
   */
  bool isConnected() {
    return connectedFlag && client.connected();
  }

  /**
   * @brief Publica float formatado em tópico MQTT.
   */
  bool publishFloat(const char* topic, float value, uint8_t decimals = 2) {
    if (!isConnected()) return false;

    char payload[32];

    if (decimals == 0) {
      snprintf(payload, sizeof(payload), "%.0f", value);
    } else if (decimals == 1) {
      snprintf(payload, sizeof(payload), "%.1f", value);
    } else {
      snprintf(payload, sizeof(payload), "%.2f", value);
    }

    return client.publish(topic, payload);
  }

  bool publishTemperature(float value) {
    return publishFloat(Config::MQTT_TOPIC_TEMP, value, 2);
  }

  bool publishPressure(float value) {
    return publishFloat(Config::MQTT_TOPIC_PRES, value, 2);
  }

  bool publishHumidity(float value) {
    return publishFloat(Config::MQTT_TOPIC_HUM, value, 2);
  }

  bool publishGas(float value) {
    return publishFloat(Config::MQTT_TOPIC_GAS, value, 2);
  }

  bool publishAltitude(float value) {
    return publishFloat(Config::MQTT_TOPIC_ALT, value, 2);
  }
};

// ============================================================================
// GERENCIADOR WI-FI
// ============================================================================

/**
 * @brief Gerencia Wi-Fi STA sem conflito com NetworkManager do ESP32 Core 3.x.
 *
 * Nome propositalmente definido como AegisWifiManager, pois ESP32 Arduino
 * Core 3.x já possui uma classe interna chamada NetworkManager.
 */
class AegisWifiManager {
private:
  Preferences prefs;

  String ssid;
  String password;

  bool hasCredentials = false;
  uint32_t lastReconnectAttemptMs = 0;

  /**
   * @brief Carrega SSID/senha de Preferences.
   */
  bool loadCredentialsFromPreferences(String& outSsid, String& outPassword) {
    prefs.begin("aegis_wifi", true);

    outSsid = prefs.getString("ssid", "");
    outPassword = prefs.getString("pass", "");

    prefs.end();

    return outSsid.length() > 0;
  }

  /**
   * @brief Resolve credenciais via constantes ou Preferences.
   */
  bool resolveCredentials() {
    if (strlen(Config::WIFI_SSID) > 0) {
      ssid = Config::WIFI_SSID;
      password = Config::WIFI_PASS;
      return true;
    }

    return loadCredentialsFromPreferences(ssid, password);
  }

  /**
   * @brief Aguarda conexão inicial com timeout.
   *
   * Este método só é usado no setup(), não no loop().
   */
  bool waitConnected(uint32_t timeoutMs) {
    const uint32_t started = millis();

    while (WiFi.status() != WL_CONNECTED && (millis() - started) < timeoutMs) {
      delay(250);
      Serial.print(".");
    }

    Serial.println();

    return WiFi.status() == WL_CONNECTED;
  }

public:
  /**
   * @brief Inicializa Wi-Fi em modo estação.
   */
  void begin() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);

    hasCredentials = resolveCredentials();

    if (!hasCredentials) {
      Serial.println("[WiFi] Nenhuma credencial configurada.");
      Serial.println("[WiFi] Preencha Config::WIFI_SSID/WIFI_PASS ou grave credenciais em Preferences.");
      return;
    }

    Serial.printf("[WiFi] Conectando em %s", ssid.c_str());

    WiFi.begin(ssid.c_str(), password.c_str());

    if (waitConnected(Config::WIFI_CONNECT_TIMEOUT_MS)) {
      Serial.printf("[WiFi] Conectado. IP=%s\n", WiFi.localIP().toString().c_str());
    } else {
      Serial.println("[WiFi] Falha inicial de conexao STA.");
    }

    lastReconnectAttemptMs = millis();
  }

  /**
   * @brief Reconexão não bloqueante.
   *
   * Deve ser chamado no loop().
   */
  void update() {
    if (!hasCredentials) return;
    if (WiFi.status() == WL_CONNECTED) return;

    const uint32_t now = millis();

    if ((now - lastReconnectAttemptMs) >= Config::WIFI_RECONNECT_INTERVAL_MS) {
      lastReconnectAttemptMs = now;

      Serial.printf("[WiFi] Reconectando em %s...\n", ssid.c_str());

      WiFi.disconnect(false, false);
      WiFi.begin(ssid.c_str(), password.c_str());
    }
  }

  bool isConnected() const {
    return WiFi.status() == WL_CONNECTED;
  }
};

// ============================================================================
// PARSER LD14P
// ============================================================================

/**
 * @brief Parser incremental do protocolo LD14P.
 *
 * O parser trabalha byte a byte, validando:
 *   - SOF 0x54
 *   - Frame 0x2C
 *   - Tamanho fixo de pacote: 47 bytes
 *   - CRC8 polinomial 0x4D
 *
 * A cada revolução detectada, retorna uma LidarScan completa.
 */
class Ld14pParser {
private:
  enum State : uint8_t {
    WAIT_SOF,
    WAIT_FRAME,
    READ_BODY
  };

  State state = WAIT_SOF;

  uint8_t packet[Config::LD_PACKET_SIZE] = {0};
  uint8_t pos = 0;

  uint16_t prevStartRaw = 0xFFFF;

  LidarScan wip;

  uint32_t speedSum = 0;
  uint16_t speedCount = 0;

  static uint16_t u16le(const uint8_t* p) {
    return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
  }

  static uint8_t crc8(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0;

    for (uint8_t i = 0; i < len; i++) {
      crc ^= data[i];

      for (uint8_t b = 0; b < 8; b++) {
        if (crc & 0x80) {
          crc = static_cast<uint8_t>((crc << 1) ^ Config::LD_CRC_POLY);
        } else {
          crc = static_cast<uint8_t>(crc << 1);
        }
      }
    }

    return crc;
  }

  void resetPacket() {
    state = WAIT_SOF;
    pos = 0;
  }

  void clearWorkingScan() {
    wip.count = 0;
    wip.speedDps = 0;
    wip.completedAtMs = 0;

    speedSum = 0;
    speedCount = 0;
  }

  void finishWorkingScan(LidarScan& out) {
    out = wip;

    out.speedDps =
      speedCount > 0
        ? static_cast<uint16_t>(speedSum / speedCount)
        : 0;

    out.completedAtMs = millis();
  }

  /**
   * @brief Decodifica um pacote válido de 47 bytes.
   */
  void decodePacket() {
    const uint16_t speed = u16le(&packet[2]);

    const uint16_t startRaw =
      u16le(&packet[4]) % Config::ANGLE_MAX_CDEG;

    const uint16_t endRaw =
      u16le(&packet[42]) % Config::ANGLE_MAX_CDEG;

    int32_t start = startRaw;
    int32_t end = endRaw;

    if (end < start) {
      end += Config::ANGLE_MAX_CDEG;
    }

    const float angleStep =
      static_cast<float>(end - start) /
      static_cast<float>(Config::LD_POINTS_PER_PACKET - 1);

    for (uint8_t i = 0; i < Config::LD_POINTS_PER_PACKET; i++) {
      const uint8_t base = 6 + i * 3;

      const uint16_t distance = u16le(&packet[base]);
      const uint8_t intensity = packet[base + 2];

      float angle =
        static_cast<float>(start) + angleStep * static_cast<float>(i);

      while (angle >= Config::ANGLE_MAX_CDEG) {
        angle -= Config::ANGLE_MAX_CDEG;
      }

      while (angle < 0.0f) {
        angle += Config::ANGLE_MAX_CDEG;
      }

      if (wip.count < Config::SCAN_MAX_POINTS) {
        LidarPoint& p = wip.points[wip.count++];

        p.angleCdeg = static_cast<uint16_t>(angle);
        p.distanceMm = distance;
        p.intensity = intensity;
      }
    }

    speedSum += speed;
    speedCount++;

    prevStartRaw = startRaw;
  }

public:
  /**
   * @brief Alimenta o parser com um byte.
   *
   * @param b Byte recebido pela UART do LD14P.
   * @param completedScan Saída com uma varredura completa quando disponível.
   * @return true se uma varredura completa foi finalizada.
   */
  bool feed(uint8_t b, LidarScan& completedScan) {
    switch (state) {
      case WAIT_SOF:
        if (b == Config::LD_SOF) {
          packet[0] = b;
          pos = 1;
          state = WAIT_FRAME;
        }
        return false;

      case WAIT_FRAME:
        if (b == Config::LD_FRAME) {
          packet[pos++] = b;
          state = READ_BODY;
        } else {
          resetPacket();
        }
        return false;

      case READ_BODY:
        packet[pos++] = b;

        if (pos < Config::LD_PACKET_SIZE) {
          return false;
        }

        if (crc8(packet, Config::LD_PACKET_SIZE - 1) != packet[Config::LD_PACKET_SIZE - 1]) {
          resetPacket();
          return false;
        }

        {
          const uint16_t startRaw =
            u16le(&packet[4]) % Config::ANGLE_MAX_CDEG;

          const bool revolutionBoundary =
            prevStartRaw != 0xFFFF &&
            startRaw < prevStartRaw &&
            wip.count > 0;

          if (revolutionBoundary) {
            finishWorkingScan(completedScan);
            clearWorkingScan();
            decodePacket();
            resetPacket();
            return true;
          }
        }

        decodePacket();
        resetPacket();
        return false;
    }

    return false;
  }
};

// ============================================================================
// INSTÂNCIAS GLOBAIS
// ============================================================================

HardwareSerial LidarSerial(2);

LidarMotorPwm lidarMotor;
VespaMotors motors;

Bme688Manager bme688Manager;
MqttManager mqttManager;
AegisWifiManager wifiManager;

Ld14pParser lidarParser;
LidarScan latestScan;

uint32_t lastMqttPublishMs = 0;
bool otaStarted = false;

// ============================================================================
// OTA
// ============================================================================

/**
 * @brief Inicializa OTA somente após conexão Wi-Fi.
 */
void startOtaIfReady() {
  if (otaStarted) return;
  if (!wifiManager.isConnected()) return;

  ArduinoOTA.setHostname(Config::OTA_HOSTNAME);
  ArduinoOTA.setPassword(Config::OTA_PASS);

  ArduinoOTA.onStart([]() {
    Serial.println("[OTA] Inicio da atualizacao.");
    motors.stop();
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("[OTA] Atualizacao finalizada.");
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("[OTA] Erro: %u\n", error);
  });

  ArduinoOTA.begin();

  otaStarted = true;

  Serial.println("[OTA] Pronto.");
}

// ============================================================================
// PUBLICAÇÃO MQTT DO BME688
// ============================================================================

/**
 * @brief Publica a leitura mais recente do BME688 nos tópicos esperados.
 */
void publishBme688ToMqtt() {
  const uint32_t now = millis();

  if ((now - lastMqttPublishMs) < Config::MQTT_PUBLISH_INTERVAL_MS) {
    return;
  }

  lastMqttPublishMs = now;

  if (!mqttManager.isConnected()) {
    return;
  }

  if (!bme688Manager.hasReading()) {
    Serial.println("[BME688] Aguardando primeira leitura valida...");
    return;
  }

  const Bme688Reading& reading = bme688Manager.latest();

  Serial.println("[MQTT-TX] Publicando dados do BME688...");

  bool ok = true;

  ok &= mqttManager.publishTemperature(reading.temperatureC);
  ok &= mqttManager.publishPressure(reading.pressureHpa);
  ok &= mqttManager.publishHumidity(reading.humidityPct);
  ok &= mqttManager.publishGas(reading.gasKohm);
  ok &= mqttManager.publishAltitude(reading.altitudeM);

  if (!ok) {
    Serial.println("[MQTT-TX] Falha em uma ou mais publicacoes.");
    return;
  }

  Serial.printf(
    "[BME688-DATA] T=%.2f C | P=%.2f hPa | H=%.2f %% | G=%.2f kOhm | A=%.2f m\n",
    reading.temperatureC,
    reading.pressureHpa,
    reading.humidityPct,
    reading.gasKohm,
    reading.altitudeM
  );
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(Config::SERIAL_BAUD);
  delay(1000);

  Serial.println();
  Serial.println("======================================================");
  Serial.println("Aegis Rover LD14P + BME688 + MQTT");
  Serial.println("Integrado com Cliente MQTT Python");
  Serial.println("======================================================");
  Serial.printf("MQTT Broker: %s:%u\n", Config::MQTT_BROKER, Config::MQTT_PORT);
  Serial.printf("Topicos MQTT: %s/*\n", Config::MQTT_TOPIC_PREFIX);
  Serial.println("Pinagem LD14P:");
  Serial.println("  BRANCO   -> RX2 / D16");
  Serial.println("  VERMELHO -> S1 / D26 / PWM");
  Serial.println("  VERDE    -> GND");
  Serial.println("  PRETO    -> 5V");
  Serial.println("======================================================");
  Serial.println();

  // Inicializa Wi-Fi apenas uma vez.
  wifiManager.begin();

  // Configura MQTT, BME688, PWM do LiDAR e UART do LiDAR.
  mqttManager.begin();
  bme688Manager.begin();

  lidarMotor.begin();

  LidarSerial.begin(
    Config::LIDAR_BAUD,
    SERIAL_8N1,
    Config::LIDAR_RX_PIN,
    Config::LIDAR_TX_PIN
  );

  motors.stop();

  // OTA só inicia se o Wi-Fi já estiver conectado.
  startOtaIfReady();

  Serial.println("[BOOT] ESP32 pronto.");
  Serial.println("[MQTT] Aguardando conexao ao broker.");
}

// ============================================================================
// LOOP PRINCIPAL
// ============================================================================

void loop() {
  // --------------------------------------------------------------------------
  // 1. Processamento contínuo do LiDAR
  // --------------------------------------------------------------------------
  while (LidarSerial.available() > 0) {
    const uint8_t b = static_cast<uint8_t>(LidarSerial.read());

    if (lidarParser.feed(b, latestScan)) {
      Serial.printf(
        "[LD14P] Nova varredura: %u pontos @ %u dps\n",
        latestScan.count,
        latestScan.speedDps
      );
    }
  }

  // --------------------------------------------------------------------------
  // 2. Serviços não bloqueantes
  // --------------------------------------------------------------------------
  wifiManager.update();
  mqttManager.update();
  bme688Manager.update();

  // --------------------------------------------------------------------------
  // 3. OTA
  // --------------------------------------------------------------------------
  startOtaIfReady();

  if (otaStarted) {
    ArduinoOTA.handle();
  }

  // --------------------------------------------------------------------------
  // 4. Publicação MQTT do BME688
  // --------------------------------------------------------------------------
  publishBme688ToMqtt();

  delay(10);
}