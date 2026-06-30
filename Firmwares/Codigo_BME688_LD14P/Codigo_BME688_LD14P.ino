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
    1. Ler telemetria ambiental do BME688.
    2. Publicar temperatura, pressão, umidade, gás e altitude via MQTT.
    3. Manter leitura contínua do LiDAR LD14P.
    4. Detectar a menor distância observada pelo LiDAR dentro de uma janela
       periódica.
    5. Publicar a menor distância detectada via MQTT.
    6. Manter OTA para atualização remota do firmware.

  ============================================================================
  PINAGEM DO LD14P
  ============================================================================

    LD14P BRANCO   -> RX2 / D16
    LD14P VERMELHO -> S1 / D26 / PWM lógico do motor
    LD14P VERDE    -> GND
    LD14P PRETO    -> 5V

  Observações importantes:
    - O ESP32 apenas recebe dados do LD14P.
    - O TX do ESP32 não é utilizado para o LD14P.
    - O fio vermelho do LD14P controla o motor via PWM lógico.
    - A pinagem foi preservada conforme solicitado.

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

    puc/iot/ld14p/menor_distancia

  Comandos MQTT -> ESP32:

    puc/iot/bme688/comando

  Comandos reconhecidos:

    delete
    termina

  ============================================================================
  NOTAS DE MANUTENÇÃO
  ============================================================================

  - O código evita String nos caminhos principais para reduzir fragmentação
    de heap.
  - O parser do LiDAR é incremental e processa byte a byte.
  - A leitura do LiDAR possui orçamento por iteração do loop para evitar que
    MQTT, OTA e Wi-Fi fiquem sem processamento.
  - O armazenamento completo dos pontos do LiDAR é opcional por macro.
  - Por padrão, o firmware armazena apenas metadados, setores e menor distância.
*/

// ============================================================================
// INCLUDES BÁSICOS
// ============================================================================

#include <Arduino.h>
#include <HardwareSerial.h>
#include <RoboCore_Vespa.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <Preferences.h>

// ============================================================================
// CONFIGURAÇÃO DO BUFFER INTERNO DO PUBSUBCLIENT
// ============================================================================
//
// O PubSubClient usa um buffer interno para montar pacotes MQTT.
// Os tópicos e payloads deste projeto são pequenos, então 128 bytes são
// suficientes para reduzir uso de RAM sem quebrar os tópicos atuais.
//
// Caso no futuro sejam publicados JSONs maiores, aumente este valor.
//
#ifndef MQTT_MAX_PACKET_SIZE
  #define MQTT_MAX_PACKET_SIZE 128
#endif

#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <stdint.h>

// Garante que o firmware seja compilado somente para arquitetura ESP32.
#ifndef ARDUINO_ARCH_ESP32
  #error Este firmware e destinado a ESP32 / RoboCore Vespa.
#endif

// ============================================================================
// OPÇÕES DE BUILD
// ============================================================================
//
// AEGIS_LIDAR_STORE_POINTS controla o uso de RAM pelo LiDAR.
//
// 0 = perfil otimizado:
//     - Não armazena todos os pontos do LiDAR.
//     - Mantém contagem, velocidade, sumário setorial e menor distância.
//     - Recomendado para este firmware atual.
//
// 1 = perfil completo:
//     - Armazena até Config::SCAN_MAX_POINTS pontos.
//     - Use se navegação/autonomia futura precisar acessar points[].
//
#ifndef AEGIS_LIDAR_STORE_POINTS
  #define AEGIS_LIDAR_STORE_POINTS 0
#endif

// ============================================================================
// CONFIGURAÇÃO CENTRALIZADA DO SISTEMA
// ============================================================================
//
// Todas as constantes principais ficam neste namespace para facilitar
// manutenção, auditoria e ajustes finos sem procurar valores espalhados.
//
namespace Config {
  // --------------------------------------------------------------------------
  // Serial de depuração
  // --------------------------------------------------------------------------
  static constexpr uint32_t SERIAL_BAUD = 115200;

  // --------------------------------------------------------------------------
  // Diagnóstico e orçamento de execução
  // --------------------------------------------------------------------------

  // Quando false, evita log de cada varredura LiDAR.
  // Isso reduz jitter, uso de CPU e tráfego na serial.
  static constexpr bool DEBUG_LIDAR_SCAN_LOG = false;

  // Limita quantos bytes da UART do LiDAR serão processados por passagem
  // do loop().
  //
  // Isso impede que o loop fique preso processando LiDAR e deixe de atender
  // MQTT, OTA, Wi-Fi e BME688.
  //
  // O pacote LD14P tem 47 bytes. Logo, 256 bytes equivalem a cerca de
  // cinco pacotes completos por iteração.
  static constexpr uint16_t LIDAR_MAX_BYTES_PER_LOOP = 256;

  // Controle de economia de energia do Wi-Fi.
  //
  // false:
  //   - Mantém comportamento anterior.
  //   - Menor latência para MQTT/OTA.
  //   - Maior consumo energético.
  //
  // true:
  //   - Permite economia de energia do Wi-Fi.
  //   - Pode aumentar latência e impactar OTA/MQTT.
  static constexpr bool WIFI_SLEEP_ENABLED = false;

  // Tamanhos máximos usados em buffers fixos.
  //
  // SSID Wi-Fi tem até 32 caracteres + '\0'.
  // Senha WPA/WPA2 pode ter até 64 caracteres + '\0'.
  static constexpr size_t WIFI_SSID_BUFFER_SIZE = 33;
  static constexpr size_t WIFI_PASS_BUFFER_SIZE = 65;

  // Buffer para comandos MQTT recebidos.
  // Os comandos atuais são curtos: delete e termina.
  static constexpr size_t MQTT_COMMAND_BUFFER_SIZE = 32;

  // Buffer para client ID MQTT:
  // "ESP32_BME688_AegisRover_" + MAC sem ':'.
  static constexpr size_t MQTT_CLIENT_ID_BUFFER_SIZE = 64;

  // --------------------------------------------------------------------------
  // Wi-Fi
  // --------------------------------------------------------------------------
  //
  // Caso não use Preferences para armazenar SSID/senha, preencha aqui:
  //
  // static const char* WIFI_SSID = "MinhaRede";
  // static const char* WIFI_PASS = "MinhaSenha";
  //
  // Se ficar vazio, o firmware tentará carregar as credenciais da NVS
  // usando Preferences no namespace "aegis_wifi".
  //
  static const char* WIFI_SSID = "MEGANET";
  static const char* WIFI_PASS = "ClaudioJ2006";

  // Timeout da tentativa inicial de conexão Wi-Fi feita no setup().
  static constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 12000;

  // Intervalo entre tentativas de reconexão no loop().
  static constexpr uint32_t WIFI_RECONNECT_INTERVAL_MS = 15000;

  // --------------------------------------------------------------------------
  // LiDAR LD14P - pinagem preservada
  // --------------------------------------------------------------------------

  // RX2 / D16: recebe dados do TX do LD14P.
  static constexpr uint8_t LIDAR_RX_PIN = 16;

  // TX não utilizado. O ESP32 apenas recebe dados do LiDAR.
  static constexpr int8_t  LIDAR_TX_PIN = -1;

  // GPIO 26 / S1: PWM lógico para motor do LD14P.
  static constexpr uint8_t LIDAR_MOTOR_PWM_PIN = 26;

  // Baudrate do LD14P.
  static constexpr uint32_t LIDAR_BAUD = 230400;

  // Configuração do PWM do motor.
  static constexpr uint32_t LIDAR_PWM_FREQ_HZ = 1000;
  static constexpr uint8_t  LIDAR_PWM_RES_BITS = 10;

  // Valor máximo do duty cycle para resolução de 10 bits.
  static constexpr uint16_t LIDAR_PWM_MAX = (1U << LIDAR_PWM_RES_BITS) - 1U;

  // Duty padrão: aproximadamente 50%.
  static constexpr uint16_t LIDAR_PWM_DEFAULT = 512;

  // Canal usado no Arduino-ESP32 Core 2.x.
  // No Core 3.x, ledcAttach() escolhe canal automaticamente.
  static constexpr uint8_t  LIDAR_PWM_LEDC_CHANNEL = 8;

  // --------------------------------------------------------------------------
  // Protocolo LD14P
  // --------------------------------------------------------------------------

  // Byte inicial do pacote.
  static constexpr uint8_t LD_SOF = 0x54;

  // Segundo byte fixo do frame.
  static constexpr uint8_t LD_FRAME = 0x2C;

  // Tamanho total de cada pacote LD14P.
  static constexpr uint8_t LD_PACKET_SIZE = 47;

  // Cada pacote contém 12 pontos.
  static constexpr uint8_t LD_POINTS_PER_PACKET = 12;

  // Polinômio CRC8 usado pelo LD14P.
  static constexpr uint8_t LD_CRC_POLY = 0x4D;

  // Ângulo máximo em centésimos de grau:
  // 360.00 graus = 36000 centésimos.
  static constexpr uint16_t ANGLE_MAX_CDEG = 36000;

  // --------------------------------------------------------------------------
  // Parâmetros gerais de leitura do LiDAR
  // --------------------------------------------------------------------------

  // Número máximo de pontos por varredura.
  // Só consome RAM significativa se AEGIS_LIDAR_STORE_POINTS = 1.
  static constexpr uint16_t SCAN_MAX_POINTS = 720;

  // Filtros usados no sumário setorial.
  static constexpr uint16_t MIN_VALID_MM = 100;
  static constexpr uint16_t MAX_VALID_MM = 8000;
  static constexpr uint8_t MIN_INTENSITY = 8;

  // --------------------------------------------------------------------------
  // Menor distância LD14P por janela de leitura
  // --------------------------------------------------------------------------

  // Intervalo para imprimir/publicar a menor distância detectada.
  static constexpr uint32_t LIDAR_NEAREST_REPORT_INTERVAL_MS = 250;

  // Filtros específicos para menor distância, seguindo o código de referência.
  // São mais permissivos que os filtros setoriais.
  static constexpr uint16_t LIDAR_NEAREST_MIN_VALID_MM = 20;
  static constexpr uint16_t LIDAR_NEAREST_MAX_VALID_MM = 12000;
  static constexpr uint8_t  LIDAR_NEAREST_MIN_INTENSITY = 1;

  // Comportamento quando nenhum obstáculo válido for detectado no intervalo.
  //
  // true:
  //   publica 0 no MQTT.
  //
  // false:
  //   não publica nada naquele intervalo.
  static constexpr bool LIDAR_NEAREST_PUBLISH_NO_DETECTION_ZERO = true;
  static constexpr uint16_t LIDAR_NEAREST_NO_DETECTION_VALUE_MM = 0;

  // --------------------------------------------------------------------------
  // Intervalos MQTT
  // --------------------------------------------------------------------------

  // Intervalo de publicação dos dados ambientais do BME688.
  static constexpr uint32_t MQTT_PUBLISH_INTERVAL_MS = 5000;

  // Intervalo mínimo entre tentativas de reconexão MQTT.
  static constexpr uint32_t MQTT_RECONNECT_DELAY_MS = 2500;

  // --------------------------------------------------------------------------
  // OTA
  // --------------------------------------------------------------------------

  // Nome exibido na rede para OTA.
  static const char* OTA_HOSTNAME = "AegisRover-LD14P";

  // Senha OTA. Em produção, recomenda-se trocar por uma senha forte.
  static const char* OTA_PASS = "AegisOTA";

  // --------------------------------------------------------------------------
  // MQTT
  // --------------------------------------------------------------------------

  static const char* MQTT_BROKER = "test.mosquitto.org";
  static constexpr uint16_t MQTT_PORT = 1883;

  // Prefixo base do client ID.
  // O MAC será anexado para reduzir risco de colisão.
  static const char* MQTT_CLIENT_ID = "ESP32_BME688_AegisRover";

  // Prefixo dos tópicos BME688.
  static const char* MQTT_TOPIC_PREFIX = "puc/iot/bme688";

  // Tópicos ambientais.
  static const char* MQTT_TOPIC_TEMP = "puc/iot/bme688/temperatura";
  static const char* MQTT_TOPIC_PRES = "puc/iot/bme688/pressao";
  static const char* MQTT_TOPIC_HUM  = "puc/iot/bme688/umidade";
  static const char* MQTT_TOPIC_GAS  = "puc/iot/bme688/gas";
  static const char* MQTT_TOPIC_ALT  = "puc/iot/bme688/altitude";

  // Tópico de comandos.
  static const char* MQTT_TOPIC_CMD  = "puc/iot/bme688/comando";

  // Tópico novo para menor distância detectada pelo LiDAR.
  static const char* MQTT_TOPIC_LIDAR_NEAREST_MM =
    "puc/iot/ld14p/menor_distancia";

  // --------------------------------------------------------------------------
  // BME688
  // --------------------------------------------------------------------------

  // Intervalo de leitura do sensor ambiental.
  static constexpr uint32_t BME688_READ_INTERVAL_MS = 5000;

  // Pressão ao nível do mar usada para estimar altitude.
  static constexpr float SEA_LEVEL_PRESSURE_HPA = 1013.25f;

  // Endereços I2C possíveis do BME688/BME680.
  static constexpr uint8_t BME688_ADDR_PRIMARY = 0x77;
  static constexpr uint8_t BME688_ADDR_SECONDARY = 0x76;
}

// ============================================================================
// UTILITÁRIOS
// ============================================================================
//
// Funções auxiliares pequenas e sem alocação dinâmica.
//
namespace Util {
  /**
   * @brief Limita um inteiro ao intervalo [low, high].
   */
  int clampInt(int value, int low, int high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
  }

  /**
   * @brief Copia uma string C para buffer fixo.
   *
   * Retorna false se:
   *   - ponteiro inválido;
   *   - buffer vazio;
   *   - string de origem maior que o destino.
   *
   * Usada para evitar String e reduzir fragmentação de heap.
   */
  bool copyCString(char* dst, size_t dstSize, const char* src) {
    if (dst == nullptr || dstSize == 0) return false;

    dst[0] = '\0';

    if (src == nullptr) return false;

    const size_t srcLen = strlen(src);

    if (srcLen >= dstSize) {
      return false;
    }

    memcpy(dst, src, srcLen + 1);
    return true;
  }

  /**
   * @brief Remove espaços ASCII no início e no fim de uma string.
   *
   * Opera diretamente no buffer recebido.
   */
  void trimAsciiInPlace(char* text) {
    if (text == nullptr || text[0] == '\0') return;

    char* start = text;

    // Avança enquanto houver espaços no início.
    while (*start != '\0' && isspace(static_cast<unsigned char>(*start))) {
      start++;
    }

    // Encontra o final da string já sem considerar espaços iniciais.
    char* end = start + strlen(start);

    // Retrocede removendo espaços finais.
    while (end > start && isspace(static_cast<unsigned char>(*(end - 1)))) {
      end--;
    }

    // Termina a string no novo final.
    *end = '\0';

    // Se a string começou depois do buffer original, desloca para o início.
    if (start != text) {
      memmove(text, start, static_cast<size_t>(end - start) + 1);
    }
  }

  /**
   * @brief Compara duas strings ASCII ignorando maiúsculas/minúsculas.
   *
   * Evita criar String apenas para usar toUpperCase().
   */
  bool equalsIgnoreCase(const char* a, const char* b) {
    if (a == nullptr || b == nullptr) return false;

    while (*a != '\0' && *b != '\0') {
      const int ca = toupper(static_cast<unsigned char>(*a));
      const int cb = toupper(static_cast<unsigned char>(*b));

      if (ca != cb) {
        return false;
      }

      a++;
      b++;
    }

    return *a == '\0' && *b == '\0';
  }
}

// ============================================================================
// ESTRUTURAS DE DADOS
// ============================================================================

/**
 * @brief Representa um ponto polar bruto do LiDAR.
 *
 * Só é armazenado em array quando AEGIS_LIDAR_STORE_POINTS = 1.
 */
struct LidarPoint {
  uint16_t angleCdeg = 0;     // Ângulo em centésimos de grau.
  uint16_t distanceMm = 0;    // Distância em milímetros.
  uint8_t intensity = 0;      // Intensidade/refletância.
};

/**
 * @brief Menor ponto válido detectado.
 *
 * Usado tanto dentro de uma varredura quanto na janela de relatório.
 */
struct LidarNearestPoint {
  bool valid = false;                 // Indica se há valor válido.
  uint16_t distanceMm = UINT16_MAX;   // Menor distância em milímetros.
  uint16_t angleCdeg = 0;             // Ângulo em centésimos de grau.
  uint8_t intensity = 0;              // Intensidade do ponto.
};

/**
 * @brief Sumário setorial simplificado da varredura LiDAR.
 *
 * Valores em milímetros.
 * Valor 0 significa que nenhum ponto válido foi observado naquele setor.
 *
 * Convenção atual:
 *   - front: 315°..360° e 0°..45°
 *   - left:  45°..135°
 *   - right: 225°..315°
 *
 * Atenção:
 *   Esta convenção assume que 0° do LiDAR aponta para frente do robô.
 *   Se o sensor estiver montado rotacionado, será necessário aplicar offset.
 */
struct SectorSummary {
  uint16_t front = 0;
  uint16_t left = 0;
  uint16_t right = 0;
  uint16_t nearest = 0;
};

/**
 * @brief Varredura LiDAR completa ou resumida.
 *
 * Em modo otimizado:
 *   - Não armazena points[].
 *   - Mantém contagem, velocidade, setores e menor ponto.
 *
 * Em modo completo:
 *   - Armazena até Config::SCAN_MAX_POINTS pontos em points[].
 */
struct LidarScan {
#if AEGIS_LIDAR_STORE_POINTS
  LidarPoint points[Config::SCAN_MAX_POINTS];
#endif

  uint16_t count = 0;         // Pontos recebidos na varredura.
  uint16_t validCount = 0;    // Pontos válidos segundo filtro setorial.
  uint16_t speedDps = 0;      // Velocidade média reportada pelo LD14P.
  uint32_t completedAtMs = 0; // Timestamp da conclusão da varredura.

  SectorSummary sectors;      // Distâncias mínimas por setor.
  LidarNearestPoint nearest;  // Menor ponto válido da varredura.
};

/**
 * @brief Leitura consolidada do BME688.
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
//
// Esta classe encapsula a configuração LEDC para acionar o motor do LD14P.
//
class LidarMotorPwm {
private:
  // Duty atual do PWM.
  uint16_t duty = Config::LIDAR_PWM_DEFAULT;

public:
  /**
   * @brief Inicializa PWM do motor do LD14P.
   *
   * Compatível com:
   *   - Arduino-ESP32 Core 3.x: ledcAttach(pin, freq, resolution)
   *   - Arduino-ESP32 Core 2.x: ledcSetup + ledcAttachPin
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
   * @brief Atualiza duty cycle do motor.
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
   * @brief Retorna duty cycle atual.
   */
  uint16_t getDuty() const {
    return duty;
  }
};

// ============================================================================
// GERENCIADOR DO SENSOR BME688
// ============================================================================
//
// Responsável por:
//   - inicializar I2C;
//   - detectar o sensor em 0x77 ou 0x76;
//   - configurar oversampling/filtro/aquecedor;
//   - executar leitura não bloqueante.
//
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
   * @brief Configura parâmetros de medição do BME688.
   */
  void configureSensor() {
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);

    // Aquecedor para medição de gás.
    // Atenção: aumenta consumo energético.
    bme.setGasHeater(320, 150);
  }

public:
  /**
   * @brief Inicializa I2C e tenta encontrar o BME688.
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

    // Força a primeira leitura logo após o setup.
    lastStartMs = millis() - Config::BME688_READ_INTERVAL_MS;

    Serial.printf("[BME688] Online em I2C 0x%02X.\n", activeAddress);
  }

  /**
   * @brief Atualiza leitura do BME688 sem bloquear o loop principal.
   */
  void update() {
    if (!available) return;

    const uint32_t now = millis();

    // Inicia nova leitura se não há leitura em andamento e o intervalo venceu.
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

    // Finaliza a leitura quando o tempo de medição já passou.
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

  /**
   * @brief Indica se já existe leitura válida.
   */
  bool hasReading() const {
    return available && reading.completedAtMs > 0;
  }

  /**
   * @brief Retorna a última leitura consolidada.
   */
  const Bme688Reading& latest() const {
    return reading;
  }
};

// ============================================================================
// GERENCIADOR MQTT
// ============================================================================
//
// Responsável por:
//   - configurar broker;
//   - manter conexão;
//   - receber comandos;
//   - publicar dados BME688;
//   - publicar menor distância LD14P.
//
class MqttManager {
private:
  WiFiClient wifiClient;
  PubSubClient client;

  uint32_t lastReconnectMs = 0;
  bool connectedFlag = false;

  /**
   * @brief Callback MQTT para comandos recebidos.
   */
  void onMessageReceived(char* topic, byte* payload, unsigned int length) {
    if (length == 0) return;

    char command[Config::MQTT_COMMAND_BUFFER_SIZE];

    const unsigned int copyLen =
      length < (sizeof(command) - 1) ? length : (sizeof(command) - 1);

    memcpy(command, payload, copyLen);
    command[copyLen] = '\0';

    Util::trimAsciiInPlace(command);

    Serial.printf("[MQTT-RX] %s = %s\n", topic, command);

    if (Util::equalsIgnoreCase(command, "DELETE")) {
      Serial.println("[CMD] DELETE recebido. Tratamento principal fica no cliente Python.");
    } else if (Util::equalsIgnoreCase(command, "TERMINA")) {
      Serial.println("[CMD] TERMINA recebido. Cliente Python deve encerrar.");
    } else {
      Serial.println("[CMD] Comando MQTT desconhecido.");
    }
  }

  /**
   * @brief Monta um client ID MQTT único usando o MAC do ESP32.
   */
  bool buildClientId(char* outClientId, size_t outSize) {
    if (outClientId == nullptr || outSize == 0) return false;

    uint8_t mac[6] = {0};
    WiFi.macAddress(mac);

    const int written = snprintf(
      outClientId,
      outSize,
      "%s_%02X%02X%02X%02X%02X%02X",
      Config::MQTT_CLIENT_ID,
      mac[0],
      mac[1],
      mac[2],
      mac[3],
      mac[4],
      mac[5]
    );

    return written > 0 && static_cast<size_t>(written) < outSize;
  }

  /**
   * @brief Tenta conectar ao broker MQTT.
   */
  bool connectToBroker() {
    Serial.printf(
      "[MQTT] Conectando a %s:%u...\n",
      Config::MQTT_BROKER,
      Config::MQTT_PORT
    );

    char clientId[Config::MQTT_CLIENT_ID_BUFFER_SIZE];

    if (!buildClientId(clientId, sizeof(clientId))) {
      connectedFlag = false;
      Serial.println("[MQTT] Falha ao montar client ID.");
      return false;
    }

    if (client.connect(clientId)) {
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
   * @brief Configura broker e callback.
   */
  void begin() {
    client.setServer(Config::MQTT_BROKER, Config::MQTT_PORT);

    client.setCallback([this](char* topic, byte* payload, unsigned int length) {
      this->onMessageReceived(topic, payload, length);
    });
  }

  /**
   * @brief Mantém MQTT vivo e tenta reconectar quando necessário.
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
   * @brief Verifica conexão MQTT.
   */
  bool isConnected() {
    return connectedFlag && client.connected();
  }

  /**
   * @brief Publica número float formatado em tópico MQTT.
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

  /**
   * @brief Publica uint16_t em tópico MQTT.
   */
  bool publishUint16(const char* topic, uint16_t value) {
    if (!isConnected()) return false;

    char payload[8];
    snprintf(payload, sizeof(payload), "%u", static_cast<unsigned int>(value));

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

  bool publishLidarNearestDistanceMm(uint16_t value) {
    return publishUint16(Config::MQTT_TOPIC_LIDAR_NEAREST_MM, value);
  }
};

// ============================================================================
// GERENCIADOR WI-FI
// ============================================================================
//
// Responsável por:
//   - carregar credenciais;
//   - iniciar Wi-Fi em modo estação;
//   - reconectar sem bloquear o loop.
//
class AegisWifiManager {
private:
  Preferences prefs;

  char ssid[Config::WIFI_SSID_BUFFER_SIZE] = {0};
  char password[Config::WIFI_PASS_BUFFER_SIZE] = {0};

  bool hasCredentials = false;
  uint32_t lastReconnectAttemptMs = 0;

  /**
   * @brief Carrega credenciais da NVS usando Preferences.
   */
  bool loadCredentialsFromPreferences() {
    ssid[0] = '\0';
    password[0] = '\0';

    if (!prefs.begin("aegis_wifi", true)) {
      Serial.println("[WiFi] Falha ao abrir Preferences.");
      return false;
    }

    const size_t ssidLen = prefs.getString(
      "ssid",
      ssid,
      sizeof(ssid)
    );

    prefs.getString(
      "pass",
      password,
      sizeof(password)
    );

    prefs.end();

    ssid[sizeof(ssid) - 1] = '\0';
    password[sizeof(password) - 1] = '\0';

    return ssidLen > 0 && ssid[0] != '\0';
  }

  /**
   * @brief Resolve credenciais via constantes ou Preferences.
   */
  bool resolveCredentials() {
    ssid[0] = '\0';
    password[0] = '\0';

    if (Config::WIFI_SSID[0] != '\0') {
      if (!Util::copyCString(ssid, sizeof(ssid), Config::WIFI_SSID)) {
        Serial.println("[WiFi] SSID em Config::WIFI_SSID excede o buffer.");
        return false;
      }

      if (!Util::copyCString(password, sizeof(password), Config::WIFI_PASS)) {
        Serial.println("[WiFi] Senha em Config::WIFI_PASS excede o buffer.");
        return false;
      }

      return true;
    }

    return loadCredentialsFromPreferences();
  }

  /**
   * @brief Aguarda conexão inicial com timeout.
   *
   * Usado somente no setup().
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

  /**
   * @brief Imprime IP sem criar String temporária.
   */
  void printLocalIp() const {
    const IPAddress ip = WiFi.localIP();

    Serial.printf(
      "[WiFi] Conectado. IP=%u.%u.%u.%u\n",
      ip[0],
      ip[1],
      ip[2],
      ip[3]
    );
  }

public:
  /**
   * @brief Inicializa Wi-Fi STA.
   */
  void begin() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(Config::WIFI_SLEEP_ENABLED);

    hasCredentials = resolveCredentials();

    if (!hasCredentials) {
      Serial.println("[WiFi] Nenhuma credencial configurada.");
      Serial.println("[WiFi] Preencha Config::WIFI_SSID/WIFI_PASS ou grave credenciais em Preferences.");
      return;
    }

    Serial.printf("[WiFi] Conectando em %s", ssid);

    WiFi.begin(ssid, password);

    if (waitConnected(Config::WIFI_CONNECT_TIMEOUT_MS)) {
      printLocalIp();
    } else {
      Serial.println("[WiFi] Falha inicial de conexao STA.");
    }

    lastReconnectAttemptMs = millis();
  }

  /**
   * @brief Reconexão Wi-Fi não bloqueante.
   */
  void update() {
    if (!hasCredentials) return;
    if (WiFi.status() == WL_CONNECTED) return;

    const uint32_t now = millis();

    if ((now - lastReconnectAttemptMs) >= Config::WIFI_RECONNECT_INTERVAL_MS) {
      lastReconnectAttemptMs = now;

      Serial.printf("[WiFi] Reconectando em %s...\n", ssid);

      WiFi.disconnect(false, false);
      WiFi.begin(ssid, password);
    }
  }

  bool isConnected() const {
    return WiFi.status() == WL_CONNECTED;
  }
};

// ============================================================================
// PARSER LD14P
// ============================================================================
//
// Parser incremental para o protocolo LD14P.
//
// Características:
//   - processa byte a byte;
//   - valida cabeçalho;
//   - valida CRC8;
//   - detecta fechamento de revolução;
//   - calcula ângulos em centésimos de grau;
//   - mantém menor distância da varredura;
//   - evita cópia pesada de buffers.
//
class Ld14pParser {
private:
  /**
   * @brief Estados do parser incremental.
   */
  enum State : uint8_t {
    WAIT_SOF,
    WAIT_FRAME,
    READ_BODY
  };

  State state = WAIT_SOF;

  // Buffer de um pacote LD14P.
  uint8_t packet[Config::LD_PACKET_SIZE] = {0};

  // Índice atual de escrita no pacote.
  uint8_t pos = 0;

  // Ângulo inicial do pacote anterior.
  // Usado para detectar virada de 360° para 0°.
  uint16_t prevStartRaw = 0xFFFF;

  // Dois buffers evitam copiar varredura completa.
  // Um buffer fica completo enquanto o outro é preenchido.
  LidarScan scanBuffers[2];
  uint8_t activeScanIndex = 0;

  // Acumuladores para velocidade média por revolução.
  uint32_t speedSum = 0;
  uint16_t speedCount = 0;

  /**
   * @brief Lê uint16 little-endian.
   */
  static uint16_t u16le(const uint8_t* p) {
    return static_cast<uint16_t>(p[0]) |
           (static_cast<uint16_t>(p[1]) << 8);
  }

  /**
   * @brief Calcula CRC8 do LD14P.
   */
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

  /**
   * @brief Retorna buffer atualmente em construção.
   */
  LidarScan& activeScan() {
    return scanBuffers[activeScanIndex];
  }

  /**
   * @brief Reinicia o parser de pacote.
   */
  void resetPacket() {
    state = WAIT_SOF;
    pos = 0;
  }

  /**
   * @brief Zera sumário setorial.
   */
  static void resetSectorSummary(SectorSummary& summary) {
    summary.front = 0;
    summary.left = 0;
    summary.right = 0;
    summary.nearest = 0;
  }

  /**
   * @brief Zera menor ponto.
   */
  static void resetNearest(LidarNearestPoint& nearest) {
    nearest.valid = false;
    nearest.distanceMm = UINT16_MAX;
    nearest.angleCdeg = 0;
    nearest.intensity = 0;
  }

  /**
   * @brief Limpa uma estrutura de varredura.
   */
  static void clearScan(LidarScan& scan) {
    scan.count = 0;
    scan.validCount = 0;
    scan.speedDps = 0;
    scan.completedAtMs = 0;

    resetSectorSummary(scan.sectors);
    resetNearest(scan.nearest);
  }

  /**
   * @brief Limpa acumuladores da varredura em construção.
   */
  void clearWorkingScanState() {
    speedSum = 0;
    speedCount = 0;
  }

  /**
   * @brief Validação para sumário setorial.
   */
  static bool isValidSectorPoint(uint16_t distanceMm, uint8_t intensity) {
    return distanceMm >= Config::MIN_VALID_MM &&
           distanceMm <= Config::MAX_VALID_MM &&
           intensity >= Config::MIN_INTENSITY;
  }

  /**
   * @brief Validação específica para menor distância publicada.
   */
  static bool isValidNearestPoint(uint16_t distanceMm, uint8_t intensity) {
    return distanceMm >= Config::LIDAR_NEAREST_MIN_VALID_MM &&
           distanceMm <= Config::LIDAR_NEAREST_MAX_VALID_MM &&
           intensity >= Config::LIDAR_NEAREST_MIN_INTENSITY;
  }

  /**
   * @brief Atualiza um mínimo armazenado.
   */
  static void updateMinimum(uint16_t& current, uint16_t candidate) {
    if (current == 0 || candidate < current) {
      current = candidate;
    }
  }

  /**
   * @brief Atualiza setores front/left/right/nearest.
   */
  static void updateSectorSummary(
    SectorSummary& summary,
    uint16_t angleCdeg,
    uint16_t distanceMm
  ) {
    updateMinimum(summary.nearest, distanceMm);

    if (angleCdeg <= 4500 || angleCdeg >= 31500) {
      updateMinimum(summary.front, distanceMm);
      return;
    }

    if (angleCdeg > 4500 && angleCdeg < 13500) {
      updateMinimum(summary.left, distanceMm);
      return;
    }

    if (angleCdeg > 22500 && angleCdeg < 31500) {
      updateMinimum(summary.right, distanceMm);
      return;
    }
  }

  /**
   * @brief Atualiza menor ponto válido.
   */
  static void updateNearestPoint(
    LidarNearestPoint& nearest,
    uint16_t angleCdeg,
    uint16_t distanceMm,
    uint8_t intensity
  ) {
    if (!nearest.valid || distanceMm < nearest.distanceMm) {
      nearest.valid = true;
      nearest.distanceMm = distanceMm;
      nearest.angleCdeg = angleCdeg;
      nearest.intensity = intensity;
    }
  }

  /**
   * @brief Adiciona um ponto decodificado à varredura atual.
   */
  void appendPoint(
    LidarScan& scan,
    uint16_t angleCdeg,
    uint16_t distanceMm,
    uint8_t intensity
  ) {
    // Conta pontos até o limite configurado.
    // No modo otimizado, não há array points[], mas a contagem permanece útil.
    if (scan.count < Config::SCAN_MAX_POINTS) {
#if AEGIS_LIDAR_STORE_POINTS
      LidarPoint& point = scan.points[scan.count];
      point.angleCdeg = angleCdeg;
      point.distanceMm = distanceMm;
      point.intensity = intensity;
#endif
      scan.count++;
    }

    // Atualiza sumário setorial com filtro mais conservador.
    if (isValidSectorPoint(distanceMm, intensity)) {
      scan.validCount++;
      updateSectorSummary(scan.sectors, angleCdeg, distanceMm);
    }

    // Atualiza menor distância com filtro específico do relatório.
    if (isValidNearestPoint(distanceMm, intensity)) {
      updateNearestPoint(
        scan.nearest,
        angleCdeg,
        distanceMm,
        intensity
      );
    }
  }

  /**
   * @brief Finaliza a varredura atual e alterna para outro buffer.
   */
  const LidarScan* finishAndSwapWorkingScan() {
    LidarScan& completed = activeScan();

    completed.speedDps =
      speedCount > 0
        ? static_cast<uint16_t>(speedSum / speedCount)
        : 0;

    completed.completedAtMs = millis();

    const LidarScan* completedPtr = &completed;

    activeScanIndex ^= 1U;

    clearScan(activeScan());
    clearWorkingScanState();

    return completedPtr;
  }

  /**
   * @brief Decodifica um pacote LD14P já validado por CRC.
   */
  void decodePacket() {
    LidarScan& scan = activeScan();

    const uint16_t speed = u16le(&packet[2]);

    const uint16_t startRaw =
      u16le(&packet[4]) % Config::ANGLE_MAX_CDEG;

    const uint16_t endRaw =
      u16le(&packet[42]) % Config::ANGLE_MAX_CDEG;

    int32_t start = static_cast<int32_t>(startRaw);
    int32_t end = static_cast<int32_t>(endRaw);

    if (end < start) {
      end += Config::ANGLE_MAX_CDEG;
    }

    const int32_t delta = end - start;
    const int32_t denominator =
      static_cast<int32_t>(Config::LD_POINTS_PER_PACKET - 1U);

    for (uint8_t i = 0; i < Config::LD_POINTS_PER_PACKET; i++) {
      const uint8_t base = 6 + i * 3;

      const uint16_t distance = u16le(&packet[base]);
      const uint8_t intensity = packet[base + 2];

      int32_t angle =
        start +
        ((delta * static_cast<int32_t>(i)) + (denominator / 2)) / denominator;

      while (angle >= Config::ANGLE_MAX_CDEG) {
        angle -= Config::ANGLE_MAX_CDEG;
      }

      while (angle < 0) {
        angle += Config::ANGLE_MAX_CDEG;
      }

      appendPoint(
        scan,
        static_cast<uint16_t>(angle),
        distance,
        intensity
      );
    }

    speedSum += speed;
    speedCount++;

    prevStartRaw = startRaw;
  }

public:
  /**
   * @brief Construtor limpa os dois buffers.
   */
  Ld14pParser() {
    clearScan(scanBuffers[0]);
    clearScan(scanBuffers[1]);
  }

  /**
   * @brief Alimenta o parser com um byte da UART.
   *
   * @return Ponteiro para varredura concluída, ou nullptr.
   */
  const LidarScan* feed(uint8_t b) {
    switch (state) {
      case WAIT_SOF:
        if (b == Config::LD_SOF) {
          packet[0] = b;
          pos = 1;
          state = WAIT_FRAME;
        }
        return nullptr;

      case WAIT_FRAME:
        if (b == Config::LD_FRAME) {
          packet[pos++] = b;
          state = READ_BODY;
        } else {
          resetPacket();
        }
        return nullptr;

      case READ_BODY:
        packet[pos++] = b;

        if (pos < Config::LD_PACKET_SIZE) {
          return nullptr;
        }

        if (crc8(packet, Config::LD_PACKET_SIZE - 1) != packet[Config::LD_PACKET_SIZE - 1]) {
          resetPacket();
          return nullptr;
        }

        {
          const uint16_t startRaw =
            u16le(&packet[4]) % Config::ANGLE_MAX_CDEG;

          const bool revolutionBoundary =
            prevStartRaw != 0xFFFF &&
            startRaw < prevStartRaw &&
            activeScan().count > 0;

          if (revolutionBoundary) {
            const LidarScan* completedScan = finishAndSwapWorkingScan();

            // O pacote atual já pertence à nova revolução.
            decodePacket();

            resetPacket();
            return completedScan;
          }
        }

        decodePacket();
        resetPacket();
        return nullptr;
    }

    return nullptr;
  }
};

// ============================================================================
// RELATÓRIO DA MENOR DISTÂNCIA DO LD14P
// ============================================================================
//
// Acumula a menor distância observada em varreduras completas dentro de uma
// janela temporal e, a cada intervalo, imprime no Serial e publica no MQTT.
//
class LidarNearestReporter {
private:
  LidarNearestPoint windowNearest;
  uint32_t lastReportMs = 0;

  /**
   * @brief Reinicia a janela de acumulação.
   */
  void resetWindow() {
    windowNearest.valid = false;
    windowNearest.distanceMm = UINT16_MAX;
    windowNearest.angleCdeg = 0;
    windowNearest.intensity = 0;
  }

  /**
   * @brief Imprime distância em mm, cm e m sem usar float.
   */
  static void printDistanceFormats(uint16_t distanceMm) {
    Serial.printf(
      "%u mm | %u.%u cm | %u.%03u m",
      static_cast<unsigned int>(distanceMm),
      static_cast<unsigned int>(distanceMm / 10),
      static_cast<unsigned int>(distanceMm % 10),
      static_cast<unsigned int>(distanceMm / 1000),
      static_cast<unsigned int>(distanceMm % 1000)
    );
  }

  /**
   * @brief Imprime ângulo em graus com duas casas, sem usar float.
   */
  static void printAngle(uint16_t angleCdeg) {
    Serial.printf(
      "%u.%02u graus",
      static_cast<unsigned int>(angleCdeg / 100),
      static_cast<unsigned int>(angleCdeg % 100)
    );
  }

  /**
   * @brief Imprime menor obstáculo acumulado na janela.
   */
  void printNearest() const {
    Serial.print("[LD14P-NEAREST] Obstaculo mais proximo: ");
    printDistanceFormats(windowNearest.distanceMm);

    Serial.print(" | angulo: ");
    printAngle(windowNearest.angleCdeg);

    Serial.printf(
      " | intensidade: %u\n",
      static_cast<unsigned int>(windowNearest.intensity)
    );
  }

public:
  LidarNearestReporter() {
    resetWindow();
  }

  /**
   * @brief Observa uma varredura concluída.
   */
  void observeScan(const LidarScan& scan) {
    if (!scan.nearest.valid) {
      return;
    }

    if (!windowNearest.valid || scan.nearest.distanceMm < windowNearest.distanceMm) {
      windowNearest = scan.nearest;
    }
  }

  /**
   * @brief Imprime/publica menor distância quando o intervalo vence.
   */
  void update(MqttManager& mqtt) {
    const uint32_t now = millis();

    if ((now - lastReportMs) < Config::LIDAR_NEAREST_REPORT_INTERVAL_MS) {
      return;
    }

    lastReportMs = now;

    if (!windowNearest.valid) {
      Serial.println("[LD14P-NEAREST] Nenhum obstaculo valido detectado no intervalo.");

      if (
        Config::LIDAR_NEAREST_PUBLISH_NO_DETECTION_ZERO &&
        mqtt.isConnected()
      ) {
        mqtt.publishLidarNearestDistanceMm(
          Config::LIDAR_NEAREST_NO_DETECTION_VALUE_MM
        );
      }

      resetWindow();
      return;
    }

    printNearest();

    if (mqtt.isConnected()) {
      const bool ok = mqtt.publishLidarNearestDistanceMm(
        windowNearest.distanceMm
      );

      if (!ok) {
        Serial.println("[LD14P-NEAREST] Falha ao publicar menor distancia no MQTT.");
      }
    }

    resetWindow();
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
const LidarScan* latestScan = nullptr;

LidarNearestReporter lidarNearestReporter;

uint32_t lastMqttPublishMs = 0;
bool otaStarted = false;

// ============================================================================
// OTA
// ============================================================================

/**
 * @brief Inicializa OTA somente quando Wi-Fi estiver conectado.
 */
void startOtaIfReady() {
  if (otaStarted) return;
  if (!wifiManager.isConnected()) return;

  ArduinoOTA.setHostname(Config::OTA_HOSTNAME);
  ArduinoOTA.setPassword(Config::OTA_PASS);

  ArduinoOTA.onStart([]() {
    Serial.println("[OTA] Inicio da atualizacao.");

    // Por segurança, para os motores durante OTA.
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
 * @brief Publica a última leitura do BME688 nos tópicos MQTT.
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

  // Pequeno atraso para estabilizar a serial durante boot.
  delay(1000);

  Serial.println();
  Serial.println("======================================================");
  Serial.println("Aegis Rover LD14P + BME688 + MQTT");
  Serial.println("Integrado com Cliente MQTT Python");
  Serial.println("Fases aplicadas:");
  Serial.println("  Fase 1 - RAM/latencia");
  Serial.println("  Fase 2 - LiDAR/RAM");
  Serial.println("  Extra   - menor distancia LD14P via MQTT");
  Serial.println("======================================================");

  Serial.printf("MQTT Broker: %s:%u\n", Config::MQTT_BROKER, Config::MQTT_PORT);
  Serial.printf("Topicos MQTT BME688: %s/*\n", Config::MQTT_TOPIC_PREFIX);

  Serial.printf(
    "Topico MQTT menor distancia LD14P: %s\n",
    Config::MQTT_TOPIC_LIDAR_NEAREST_MM
  );

  Serial.println("Pinagem LD14P:");
  Serial.println("  BRANCO   -> RX2 / D16");
  Serial.println("  VERMELHO -> S1 / D26 / PWM");
  Serial.println("  VERDE    -> GND");
  Serial.println("  PRETO    -> 5V");

  Serial.printf(
    "[LD14P] Armazenamento de pontos: %s\n",
    AEGIS_LIDAR_STORE_POINTS ? "habilitado" : "desabilitado"
  );

  Serial.println("======================================================");
  Serial.println();

  // Inicializa Wi-Fi uma única vez.
  wifiManager.begin();

  // Configura MQTT, mas a conexão real é mantida no loop().
  mqttManager.begin();

  // Inicializa BME688.
  bme688Manager.begin();

  // Inicializa PWM do motor do LD14P.
  lidarMotor.begin();

  // Inicializa UART dedicada ao LD14P.
  LidarSerial.begin(
    Config::LIDAR_BAUD,
    SERIAL_8N1,
    Config::LIDAR_RX_PIN,
    Config::LIDAR_TX_PIN
  );

  // Garante motores parados no boot.
  motors.stop();

  // OTA inicia apenas se Wi-Fi já estiver conectado.
  startOtaIfReady();

  Serial.println("[BOOT] ESP32 pronto.");
  Serial.println("[MQTT] Aguardando conexao ao broker.");
}

// ============================================================================
// LOOP PRINCIPAL
// ============================================================================

void loop() {
  // --------------------------------------------------------------------------
  // 1. Processamento contínuo do LiDAR com orçamento por iteração
  // --------------------------------------------------------------------------
  //
  // Esta etapa roda primeiro porque a UART do LD14P é rápida.
  // O limite LIDAR_MAX_BYTES_PER_LOOP evita que o loop fique preso aqui.
  //
  uint16_t lidarBytesProcessed = 0;

  while (
    LidarSerial.available() > 0 &&
    lidarBytesProcessed < Config::LIDAR_MAX_BYTES_PER_LOOP
  ) {
    const uint8_t b = static_cast<uint8_t>(LidarSerial.read());
    lidarBytesProcessed++;

    const LidarScan* completedScan = lidarParser.feed(b);

    if (completedScan != nullptr) {
      latestScan = completedScan;

      // Alimenta a janela de menor distância.
      lidarNearestReporter.observeScan(*latestScan);

      // Log detalhado opcional.
      if (Config::DEBUG_LIDAR_SCAN_LOG) {
        Serial.printf(
          "[LD14P] Nova varredura: %u pontos, %u validos @ %u dps | "
          "front=%u mm left=%u mm right=%u mm nearest=%u mm\n",
          static_cast<unsigned int>(latestScan->count),
          static_cast<unsigned int>(latestScan->validCount),
          static_cast<unsigned int>(latestScan->speedDps),
          static_cast<unsigned int>(latestScan->sectors.front),
          static_cast<unsigned int>(latestScan->sectors.left),
          static_cast<unsigned int>(latestScan->sectors.right),
          static_cast<unsigned int>(latestScan->sectors.nearest)
        );
      }
    }
  }

  // --------------------------------------------------------------------------
  // 2. Serviços não bloqueantes
  // --------------------------------------------------------------------------
  wifiManager.update();
  mqttManager.update();
  bme688Manager.update();

  // --------------------------------------------------------------------------
  // 3. Relatório periódico da menor distância LD14P
  // --------------------------------------------------------------------------
  //
  // Publica no tópico:
  //   puc/iot/ld14p/menor_distancia
  //
  lidarNearestReporter.update(mqttManager);

  // --------------------------------------------------------------------------
  // 4. OTA
  // --------------------------------------------------------------------------
  startOtaIfReady();

  if (otaStarted) {
    ArduinoOTA.handle();
  }

  // --------------------------------------------------------------------------
  // 5. Publicação MQTT do BME688
  // --------------------------------------------------------------------------
  publishBme688ToMqtt();

  // Cede CPU ao scheduler/Wi-Fi sem impor delay fixo.
  yield();
}