from __future__ import annotations

import json
import math
import os
import queue
import re
import threading
import time
import uuid
from dataclasses import dataclass
from getpass import getpass
from pathlib import Path
from typing import Any

try:
    from dotenv import load_dotenv
except ModuleNotFoundError:
    load_dotenv = None

try:
    import mysql.connector
except ModuleNotFoundError as exc:
    raise SystemExit(
        "Dependência ausente: mysql-connector-python. "
        "Instale com: python -m pip install mysql-connector-python"
    ) from exc

try:
    import paho.mqtt.client as mqtt
except ModuleNotFoundError as exc:
    raise SystemExit(
        "Dependência ausente: paho-mqtt. "
        "Instale com: python -m pip install paho-mqtt"
    ) from exc


# =============================================================================
# CARREGAMENTO OPCIONAL DO .env
# =============================================================================

if load_dotenv:
    arquivo_atual = Path(__file__).resolve()

    candidatos_env = [
        arquivo_atual.parent / ".env",
        arquivo_atual.parent.parent / ".env",
    ]

    for candidato in candidatos_env:
        if candidato.exists():
            load_dotenv(candidato)
            break


# =============================================================================
# UTILITÁRIOS
# =============================================================================

NUMERIC_RE = re.compile(
    r"[-+]?(?:\d+(?:[.,]\d*)?|[.,]\d+)(?:[eE][-+]?\d+)?"
)


def validar_identificador_mysql(valor: str, nome: str) -> str:
    if not re.fullmatch(r"[A-Za-z0-9_]+", valor):
        raise ValueError(
            f"{nome} deve conter apenas letras, números e underscore: {valor!r}"
        )

    return valor


def ler_bool_env(nome: str, padrao: bool) -> bool:
    valor = os.getenv(nome)

    if valor is None:
        return padrao

    return valor.strip().lower() in {"1", "true", "yes", "sim", "s"}


def extrair_float(payload: str) -> float:
    texto = payload.strip()

    try:
        dados_json: Any = json.loads(texto)
    except json.JSONDecodeError:
        dados_json = None

    if isinstance(dados_json, dict):
        for chave in ("value", "valor", "temperature", "pressure", "humidity", "gas", "altitude"):
            if chave in dados_json:
                return float(str(dados_json[chave]).replace(",", "."))

    match = NUMERIC_RE.search(texto)

    if not match:
        raise ValueError(f"payload sem valor numérico: {payload!r}")

    numero = float(match.group(0).replace(",", "."))

    if not math.isfinite(numero):
        raise ValueError(f"valor numérico não finito: {payload!r}")

    return numero


def mqtt_connection_succeeded(reason_code: Any) -> bool:
    if reason_code == 0:
        return True

    return str(reason_code).strip().lower() in {"0", "success"}


# =============================================================================
# CONFIGURAÇÃO
# =============================================================================

@dataclass(frozen=True)
class Config:
    mqtt_brokers: tuple[str, ...]
    mqtt_port: int
    mqtt_keepalive: int
    mqtt_client_id_base: str
    mqtt_topic_prefix: str

    mysql_host: str
    mysql_database: str
    mysql_user: str
    mysql_password: str | None
    tabela_raw: str

    sessao_id: str
    classe: str | None
    fase: str | None
    sensor_index: int
    sensor_id: int
    label_tag: int
    error_code: int
    heater_profile_id: str
    duty_cycle_profile_id: str
    board_type: str
    board_id: str
    firmware_version: str | None

    gas_input_unit: str
    store_gas_as_ohm: bool

    @classmethod
    def from_env(cls) -> Config:
        mqtt_brokers = tuple(
            broker.strip()
            for broker in os.getenv(
                "MQTT_BROKERS",
                "broker.emqx.io,test.mosquitto.org",
            ).split(",")
            if broker.strip()
        )

        sessao_padrao = time.strftime("aegis_bme688_%Y%m%d_%H%M%S")

        return cls(
            mqtt_brokers=mqtt_brokers,
            mqtt_port=int(os.getenv("MQTT_PORT", "1883")),
            mqtt_keepalive=int(os.getenv("MQTT_KEEPALIVE", "60")),
            mqtt_client_id_base=os.getenv(
                "MQTT_CLIENT_ID",
                "cfsj_aegis_bme688_raw_collector",
            ),
            mqtt_topic_prefix=os.getenv(
                "MQTT_TOPIC_PREFIX",
                "cfsj/aegis/bme688",
            ).strip("/"),

            mysql_host=os.getenv("IOT_MYSQL_HOST", "localhost"),
            mysql_database=validar_identificador_mysql(
                os.getenv("IOT_MYSQL_DATABASE", "IoT"),
                "IOT_MYSQL_DATABASE",
            ),
            mysql_user=os.getenv("IOT_MYSQL_USER", "root"),
            mysql_password=os.getenv("IOT_MYSQL_PASSWORD"),
            tabela_raw=validar_identificador_mysql(
                os.getenv("IOT_MYSQL_TABLE_BOSCH_RAW", "dados_bme688_bosch_raw"),
                "IOT_MYSQL_TABLE_BOSCH_RAW",
            ),

            sessao_id=os.getenv("BME688_SESSION_ID", sessao_padrao),
            classe=os.getenv("BME688_CLASSE") or None,
            fase=os.getenv("BME688_FASE", "coleta_mqtt_scalar"),
            sensor_index=int(os.getenv("BME688_SENSOR_INDEX", "0")),
            sensor_id=int(os.getenv("BME688_SENSOR_ID", "0")),
            label_tag=int(os.getenv("BME688_LABEL_TAG", "0")),
            error_code=int(os.getenv("BME688_ERROR_CODE", "0")),
            heater_profile_id=os.getenv("BME688_HEATER_PROFILE_ID", "heater_320c_150ms"),
            duty_cycle_profile_id=os.getenv("BME688_DUTY_CYCLE_PROFILE_ID", "period_2000ms"),
            board_type=os.getenv("BME688_BOARD_TYPE", "esp32_bme688_mqtt_scalar"),
            board_id=os.getenv("BME688_BOARD_ID", "AegisRover_BME688_01"),
            firmware_version=os.getenv("BME688_FIRMWARE_VERSION", "mqtt_failover_menu_serial"),

            gas_input_unit=os.getenv("BME688_GAS_INPUT_UNIT", "kohm").strip().lower(),
            store_gas_as_ohm=ler_bool_env("BME688_STORE_GAS_AS_OHM", True),
        )


# =============================================================================
# MODELOS MQTT
# =============================================================================

@dataclass(frozen=True)
class SensorTopic:
    topic: str
    coluna_buffer: str
    unidade: str


@dataclass(frozen=True)
class MqttEvent:
    broker: str
    topic: str
    payload: str


# =============================================================================
# AGREGADOR DE TÓPICOS ESCALARES
# =============================================================================

class BufferAgregadorScalar:
    """
    Reagrupa os cinco tópicos escalares publicados pelo ESP32 em uma única amostra.

    O firmware publica:

    - temperature
    - pressure
    - humidity
    - gas
    - altitude

    Esta classe consolida esses valores em uma linha compatível com a tabela raw.
    """

    def __init__(self, colunas_esperadas: int) -> None:
        self.buffer_atual: dict[str, float] = {}
        self.colunas_esperadas = colunas_esperadas
        self.broker_atual: str | None = None

    def adicionar(
        self,
        broker: str,
        coluna: str,
        valor: float,
    ) -> dict[str, float] | None:
        if self.broker_atual is None:
            self.broker_atual = broker

        if broker != self.broker_atual:
            self.buffer_atual.clear()
            self.broker_atual = broker

        if coluna in self.buffer_atual:
            self.buffer_atual = {coluna: valor}
            return None

        self.buffer_atual[coluna] = valor

        if len(self.buffer_atual) == self.colunas_esperadas:
            amostra = self.buffer_atual.copy()
            self.buffer_atual.clear()
            return amostra

        return None

    def limpar(self) -> None:
        self.buffer_atual.clear()
        self.broker_atual = None


# =============================================================================
# BANCO DE DADOS
# =============================================================================

INSERT_COLUMNS = [
    "sessao_id",
    "classe",
    "fase",
    "sensor_index",
    "sensor_id",
    "timestamp_since_poweron",
    "real_time_clock",
    "temperature",
    "pressure",
    "relative_humidity",
    "resistance_gassensor",
    "altitude",
    "heater_profile_step_index",
    "scanning_enabled",
    "scanning_cycle_index",
    "label_tag",
    "error_code",
    "heater_profile_id",
    "duty_cycle_profile_id",
    "board_type",
    "board_id",
    "firmware_version",
    "mqtt_broker",
]


class BancoBoschRaw:
    def __init__(self, config: Config) -> None:
        self.config = config
        self.conexao: mysql.connector.MySQLConnection | None = None
        self.lock = threading.RLock()
        self.conectar()
        self.criar_tabela()
        self.garantir_colunas_adicionais()

    def conectar(self) -> None:
        with self.lock:
            if self.conexao is not None and self.conexao.is_connected():
                return

            senha = self.config.mysql_password

            if senha is None:
                senha = getpass(
                    f"Senha MySQL para {self.config.mysql_user}@{self.config.mysql_host}: "
                )

            self.conexao = mysql.connector.connect(
                host=self.config.mysql_host,
                user=self.config.mysql_user,
                password=senha,
            )

            cursor = self.conexao.cursor()

            cursor.execute(
                f"""
                CREATE DATABASE IF NOT EXISTS `{self.config.mysql_database}`
                CHARACTER SET utf8mb4
                COLLATE utf8mb4_unicode_ci
                """
            )

            self.conexao.database = self.config.mysql_database
            cursor.close()
            self.conexao.commit()

    def criar_tabela(self) -> None:
        query = f"""
            CREATE TABLE IF NOT EXISTS `{self.config.tabela_raw}` (
                id BIGINT AUTO_INCREMENT PRIMARY KEY,
                sessao_id VARCHAR(64) NOT NULL,
                classe VARCHAR(64) NULL,
                fase VARCHAR(32) NULL,
                sensor_index INT NOT NULL DEFAULT 0,
                sensor_id BIGINT NOT NULL,
                timestamp_since_poweron BIGINT NOT NULL,
                real_time_clock BIGINT NOT NULL,
                temperature DOUBLE NOT NULL,
                pressure DOUBLE NOT NULL,
                relative_humidity DOUBLE NOT NULL,
                resistance_gassensor DOUBLE NOT NULL,
                altitude DOUBLE NULL,
                heater_profile_step_index INT NOT NULL,
                scanning_enabled BOOLEAN NOT NULL DEFAULT TRUE,
                scanning_cycle_index INT NOT NULL,
                label_tag INT NOT NULL DEFAULT 0,
                error_code INT NOT NULL DEFAULT 0,
                heater_profile_id VARCHAR(64) NOT NULL DEFAULT 'heater_320c_150ms',
                duty_cycle_profile_id VARCHAR(64) NOT NULL DEFAULT 'period_2000ms',
                board_type VARCHAR(64) NOT NULL DEFAULT 'esp32_bme688_mqtt_scalar',
                board_id VARCHAR(128) NOT NULL DEFAULT 'AegisRover_BME688_01',
                firmware_version VARCHAR(64) NULL,
                mqtt_broker VARCHAR(128) NULL,
                recebido_em TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
                INDEX idx_raw_sessao (sessao_id),
                INDEX idx_raw_classe (classe),
                INDEX idx_raw_poweron (timestamp_since_poweron),
                INDEX idx_raw_rtc (real_time_clock),
                INDEX idx_raw_cycle (scanning_cycle_index, heater_profile_step_index),
                INDEX idx_raw_broker (mqtt_broker)
            ) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4
        """

        with self.lock:
            self.conectar()
            assert self.conexao is not None

            cursor = self.conexao.cursor()
            cursor.execute(query)
            cursor.close()
            self.conexao.commit()

    def coluna_existe(self, coluna: str) -> bool:
        self.conectar()
        assert self.conexao is not None

        cursor = self.conexao.cursor()
        cursor.execute(
            f"SHOW COLUMNS FROM `{self.config.tabela_raw}` LIKE %s",
            (coluna,),
        )

        existe = cursor.fetchone() is not None
        cursor.close()

        return existe

    def garantir_coluna(self, coluna: str, definicao_sql: str) -> None:
        with self.lock:
            if self.coluna_existe(coluna):
                return

            assert self.conexao is not None

            cursor = self.conexao.cursor()
            cursor.execute(
                f"ALTER TABLE `{self.config.tabela_raw}` "
                f"ADD COLUMN `{coluna}` {definicao_sql}"
            )
            cursor.close()
            self.conexao.commit()

            print(f"[BD] Coluna adicionada: {coluna}")

    def garantir_colunas_adicionais(self) -> None:
        self.garantir_coluna("altitude", "DOUBLE NULL")
        self.garantir_coluna("mqtt_broker", "VARCHAR(128) NULL")

    def inserir_amostra(self, amostra: dict[str, Any]) -> int:
        colunas = ", ".join(f"`{coluna}`" for coluna in INSERT_COLUMNS)
        placeholders = ", ".join(["%s"] * len(INSERT_COLUMNS))
        valores = tuple(amostra.get(coluna) for coluna in INSERT_COLUMNS)

        with self.lock:
            self.conectar()
            assert self.conexao is not None

            cursor = self.conexao.cursor()

            cursor.execute(
                f"""
                INSERT INTO `{self.config.tabela_raw}`
                ({colunas})
                VALUES ({placeholders})
                """,
                valores,
            )

            self.conexao.commit()
            inserted_id = int(cursor.lastrowid)
            cursor.close()

            return inserted_id

    def exibir_ultimas(self, limite: int = 10) -> None:
        with self.lock:
            self.conectar()
            assert self.conexao is not None

            cursor = self.conexao.cursor()

            cursor.execute(
                f"""
                SELECT
                    id,
                    recebido_em,
                    sessao_id,
                    classe,
                    fase,
                    mqtt_broker,
                    sensor_id,
                    timestamp_since_poweron,
                    heater_profile_step_index,
                    scanning_cycle_index,
                    temperature,
                    pressure,
                    relative_humidity,
                    resistance_gassensor,
                    altitude,
                    error_code
                FROM `{self.config.tabela_raw}`
                ORDER BY id DESC
                LIMIT %s
                """,
                (limite,),
            )

            linhas = cursor.fetchall()
            cursor.close()

        print("\n" + "=" * 150)
        print(f"{self.config.mysql_database}.{self.config.tabela_raw}")
        print("=" * 150)

        for linha in linhas:
            print(" | ".join(str(valor) for valor in linha))

        print("=" * 150 + "\n")

    def apagar_leituras(self) -> None:
        with self.lock:
            self.conectar()
            assert self.conexao is not None

            cursor = self.conexao.cursor()
            cursor.execute(f"TRUNCATE TABLE `{self.config.tabela_raw}`")
            cursor.close()
            self.conexao.commit()

        print(f"[BD] Tabela `{self.config.tabela_raw}` truncada com sucesso.")

    def fechar(self) -> None:
        with self.lock:
            if self.conexao is not None and self.conexao.is_connected():
                self.conexao.close()
                print("[BD] Conexão encerrada.")


# =============================================================================
# COLETOR MQTT -> RAW COMPATÍVEL
# =============================================================================

class ColetorBME688ScalarRaw:
    def __init__(self, config: Config, banco: BancoBoschRaw) -> None:
        self.config = config
        self.banco = banco
        self.fila: queue.Queue[MqttEvent | None] = queue.Queue()
        self.stop_event = threading.Event()
        self.clientes: list[mqtt.Client] = []

        self.topicos = self._criar_topicos()
        self.agregador = BufferAgregadorScalar(
            colunas_esperadas=len(self.topicos),
        )

        self.start_monotonic = time.monotonic()
        self.scanning_cycle_index = 0

        self.thread_processador = threading.Thread(
            target=self._processar_fila,
            daemon=True,
        )

    def _criar_topicos(self) -> dict[str, SensorTopic]:
        prefix = self.config.mqtt_topic_prefix

        return {
            f"{prefix}/temperature": SensorTopic(
                topic=f"{prefix}/temperature",
                coluna_buffer="temperature",
                unidade="°C",
            ),
            f"{prefix}/pressure": SensorTopic(
                topic=f"{prefix}/pressure",
                coluna_buffer="pressure",
                unidade="hPa",
            ),
            f"{prefix}/humidity": SensorTopic(
                topic=f"{prefix}/humidity",
                coluna_buffer="relative_humidity",
                unidade="%",
            ),
            f"{prefix}/gas": SensorTopic(
                topic=f"{prefix}/gas",
                coluna_buffer="resistance_gassensor",
                unidade="kΩ",
            ),
            f"{prefix}/altitude": SensorTopic(
                topic=f"{prefix}/altitude",
                coluna_buffer="altitude",
                unidade="m",
            ),
        }

    @property
    def status_topic(self) -> str:
        return f"{self.config.mqtt_topic_prefix}/status"

    def conectar(self) -> None:
        self.thread_processador.start()

        for broker in self.config.mqtt_brokers:
            cliente = self._criar_cliente(broker)

            print(f"[MQTT][{broker}] Conectando em {broker}:{self.config.mqtt_port}")

            cliente.connect_async(
                broker,
                self.config.mqtt_port,
                self.config.mqtt_keepalive,
            )

            cliente.loop_start()
            self.clientes.append(cliente)

    def encerrar(self) -> None:
        self.stop_event.set()
        self.fila.put(None)

        for cliente in self.clientes:
            try:
                cliente.loop_stop()
                cliente.disconnect()
            except Exception:
                pass

        if self.thread_processador.is_alive():
            self.thread_processador.join(timeout=2.0)

    def _criar_cliente(self, broker: str) -> mqtt.Client:
        broker_sanitizado = re.sub(r"[^A-Za-z0-9_]", "_", broker)

        client_id = (
            f"{self.config.mqtt_client_id_base}_"
            f"{broker_sanitizado}_"
            f"{uuid.uuid4().hex[:8]}"
        )

        try:
            client = mqtt.Client(
                mqtt.CallbackAPIVersion.VERSION2,
                client_id=client_id,
            )
        except AttributeError:
            client = mqtt.Client(client_id=client_id)

        client.user_data_set({"broker": broker})

        client.on_connect = self._on_connect
        client.on_message = self._on_message
        client.on_disconnect = self._on_disconnect

        client.reconnect_delay_set(min_delay=2, max_delay=30)

        return client

    def _on_connect(
        self,
        client: mqtt.Client,
        userdata: Any,
        flags: Any,
        rc: Any,
        properties: Any = None,
    ) -> None:
        broker = userdata["broker"]

        if not mqtt_connection_succeeded(rc):
            print(f"[MQTT][{broker}] Falha de conexão: {rc}")
            return

        print(f"[MQTT][{broker}] Conectado.")

        for topico in self.topicos.values():
            client.subscribe(topico.topic)
            print(f"[MQTT][{broker}] Subscribe: {topico.topic}")

        client.subscribe(self.status_topic)
        print(f"[MQTT][{broker}] Subscribe: {self.status_topic}")

    def _on_disconnect(self, client: mqtt.Client, userdata: Any, *args: Any) -> None:
        broker = userdata["broker"]
        motivo = args[-1] if args else "desconhecido"
        print(f"[MQTT][{broker}] Desconectado: {motivo}")

    def _on_message(
        self,
        client: mqtt.Client,
        userdata: Any,
        msg: mqtt.MQTTMessage,
    ) -> None:
        broker = userdata["broker"]
        payload = msg.payload.decode("utf-8", errors="replace").strip()

        self.fila.put(
            MqttEvent(
                broker=broker,
                topic=msg.topic,
                payload=payload,
            )
        )

    def _processar_fila(self) -> None:
        while not self.stop_event.is_set():
            try:
                evento = self.fila.get(timeout=0.5)
            except queue.Empty:
                continue

            if evento is None:
                self.fila.task_done()
                break

            try:
                self._processar_evento(evento)
            finally:
                self.fila.task_done()

    def _processar_evento(self, evento: MqttEvent) -> None:
        if evento.topic == self.status_topic:
            print(f"[STATUS][{evento.broker}] {evento.payload}")
            return

        topico_info = self.topicos.get(evento.topic)

        if topico_info is None:
            return

        try:
            valor = extrair_float(evento.payload)

            if topico_info.coluna_buffer == "resistance_gassensor":
                valor = self._normalizar_gas(valor)

            print(
                f"[MQTT][{evento.broker}] "
                f"{evento.topic} = {valor:.2f} "
                f"{'Ω' if topico_info.coluna_buffer == 'resistance_gassensor' else topico_info.unidade}"
            )

            amostra_parcial = self.agregador.adicionar(
                broker=evento.broker,
                coluna=topico_info.coluna_buffer,
                valor=valor,
            )

            if amostra_parcial is None:
                return

            amostra_sql = self._montar_amostra_sql(
                broker=evento.broker,
                dados=amostra_parcial,
            )

            inserted_id = self.banco.inserir_amostra(amostra_sql)

            print(
                f"[RAW] id={inserted_id} "
                f"sessao={amostra_sql['sessao_id']} "
                f"cycle={amostra_sql['scanning_cycle_index']} "
                f"temp={amostra_sql['temperature']:.2f} "
                f"press={amostra_sql['pressure']:.2f} "
                f"hum={amostra_sql['relative_humidity']:.2f} "
                f"gas={amostra_sql['resistance_gassensor']:.2f} "
                f"alt={amostra_sql['altitude']:.2f}"
            )

        except Exception as exc:
            print(
                f"[RAW-REJEITADO] broker={evento.broker} "
                f"topic={evento.topic} erro={exc} payload={evento.payload[:240]}"
            )

    def _normalizar_gas(self, valor: float) -> float:
        if not self.config.store_gas_as_ohm:
            return valor

        if self.config.gas_input_unit in {"kohm", "kω", "kiloohm", "kilo-ohm"}:
            return valor * 1000.0

        if self.config.gas_input_unit in {"ohm", "ω"}:
            return valor

        raise ValueError(
            f"Unidade de gás inválida em BME688_GAS_INPUT_UNIT: "
            f"{self.config.gas_input_unit!r}"
        )

    def _montar_amostra_sql(
        self,
        broker: str,
        dados: dict[str, float],
    ) -> dict[str, Any]:
        self.scanning_cycle_index += 1

        timestamp_since_poweron = int(
            (time.monotonic() - self.start_monotonic) * 1000
        )

        real_time_clock = int(time.time() * 1000)

        return {
            "sessao_id": self.config.sessao_id,
            "classe": self.config.classe,
            "fase": self.config.fase,
            "sensor_index": self.config.sensor_index,
            "sensor_id": self.config.sensor_id,
            "timestamp_since_poweron": timestamp_since_poweron,
            "real_time_clock": real_time_clock,
            "temperature": dados["temperature"],
            "pressure": dados["pressure"],
            "relative_humidity": dados["relative_humidity"],
            "resistance_gassensor": dados["resistance_gassensor"],
            "altitude": dados["altitude"],
            "heater_profile_step_index": 0,
            "scanning_enabled": True,
            "scanning_cycle_index": self.scanning_cycle_index,
            "label_tag": self.config.label_tag,
            "error_code": self.config.error_code,
            "heater_profile_id": self.config.heater_profile_id,
            "duty_cycle_profile_id": self.config.duty_cycle_profile_id,
            "board_type": self.config.board_type,
            "board_id": self.config.board_id,
            "firmware_version": self.config.firmware_version,
            "mqtt_broker": broker,
        }


# =============================================================================
# PAINEL CLI
# =============================================================================

def imprimir_ajuda() -> None:
    print(
        """
Comandos:
  ver       Exibe as últimas amostras gravadas
  apagar    Apaga todos os registros da tabela raw
  ajuda     Mostra este painel
  config    Mostra a configuração ativa
  sair      Encerra o coletor
"""
    )


def imprimir_config(config: Config) -> None:
    print("\n" + "=" * 80)
    print("CONFIGURAÇÃO ATIVA")
    print("=" * 80)
    print(f"MQTT brokers: {', '.join(config.mqtt_brokers)}")
    print(f"MQTT porta: {config.mqtt_port}")
    print(f"MQTT prefixo: {config.mqtt_topic_prefix}")
    print(f"MySQL host: {config.mysql_host}")
    print(f"MySQL database: {config.mysql_database}")
    print(f"MySQL table raw: {config.tabela_raw}")
    print(f"Sessão: {config.sessao_id}")
    print(f"Board ID: {config.board_id}")
    print(f"Gas input unit: {config.gas_input_unit}")
    print(f"Store gas as ohm: {config.store_gas_as_ohm}")
    print("=" * 80 + "\n")


def main() -> None:
    config = Config.from_env()
    banco = BancoBoschRaw(config)
    coletor = ColetorBME688ScalarRaw(config, banco)

    print("AegisRover BME688 MQTT Scalar -> Bosch Raw Compatible Collector")
    print(f"Banco: {config.mysql_database}")
    print(f"Tabela raw: {config.tabela_raw}")
    print(f"Prefixo MQTT: {config.mqtt_topic_prefix}")

    imprimir_config(config)
    imprimir_ajuda()

    try:
        coletor.conectar()

        while True:
            comando = input("> ").strip().lower()

            if comando in {"sair", "exit", "quit"}:
                break

            if comando in {"ver", "exibir", "listar"}:
                banco.exibir_ultimas()

            elif comando in {"apagar", "delete", "clear", "truncate"}:
                confirmacao = input(
                    "Confirmar TRUNCATE da tabela? Digite SIM: "
                ).strip().upper()

                if confirmacao == "SIM":
                    banco.apagar_leituras()
                else:
                    print("[BD] Operação cancelada.")

            elif comando in {"ajuda", "help", "h", "menu"}:
                imprimir_ajuda()

            elif comando in {"config", "conf"}:
                imprimir_config(config)

            elif comando:
                print(f"[APP] Comando não reconhecido: {comando!r}")

    except KeyboardInterrupt:
        print("\n[APP] Finalizado via Ctrl+C.")

    finally:
        coletor.encerrar()
        banco.fechar()
        print("[APP] Sistema encerrado.")


if __name__ == "__main__":
    main()