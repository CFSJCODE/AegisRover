from __future__ import annotations

import json
import os
import queue
import re
import threading
import uuid
from dataclasses import dataclass
from datetime import datetime
from getpass import getpass
from pathlib import Path
from typing import Any

try:
    from dotenv import load_dotenv
except ModuleNotFoundError:
    load_dotenv = None

try:
    import mysql.connector
    from mysql.connector import Error as MySQLError
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
# Aegis Rover - ESP32 BME688 + LD14P -> MQTT -> MySQL
# =============================================================================
#
# Este script foi adequado aos tópicos publicados pelo firmware ESP32:
#
#   puc/iot/bme688/temperatura
#   puc/iot/bme688/pressao
#   puc/iot/bme688/umidade
#   puc/iot/bme688/gas
#   puc/iot/bme688/altitude
#   puc/iot/ld14p/menor_distancia
#
# O banco usa APENAS UMA TABELA operacional, com uma linha por amostra completa
# do BME688. A menor distância do LiDAR mais recente é anexada à mesma linha.
#
# Colunas principais da tabela:
#
#   temperatura
#   pressao
#   umidade
#   gas
#   altitude
#   menor_distancia_mm
#   menor_distancia_cm
#   menor_distancia_m
#
# Observação:
#   O firmware ESP32 pode publicar 0 em puc/iot/ld14p/menor_distancia quando
#   não há obstáculo válido. Por padrão, este script trata esse 0 como NULL
#   para não registrar "0 cm" como obstáculo real.
# =============================================================================


# =============================================================================
# CONFIGURAÇÃO DE AMBIENTE
# =============================================================================

BASE_DIR = Path(__file__).resolve().parent

if load_dotenv:
    for env_file in (BASE_DIR / ".env", BASE_DIR.parent / ".env"):
        if env_file.exists():
            load_dotenv(env_file)
            break


NUMERIC_RE = re.compile(
    r"[-+]?(?:\d+(?:[.,]\d*)?|[.,]\d+)(?:[eE][-+]?\d+)?"
)


# =============================================================================
# UTILITÁRIOS
# =============================================================================

def validar_identificador_mysql(valor: str, nome: str) -> str:
    """
    Valida nomes de banco/tabela para uso seguro em SQL com interpolação.

    MySQL não aceita placeholders para nomes de tabela, então esta validação
    evita injeção quando o nome vem do .env.
    """
    valor = valor.strip()

    if not re.fullmatch(r"[A-Za-z0-9_]+", valor):
        raise ValueError(
            f"{nome} deve conter apenas letras, números e underscore: {valor!r}"
        )

    return valor


def env_bool(nome: str, padrao: bool) -> bool:
    valor = os.getenv(nome)

    if valor is None:
        return padrao

    return valor.strip().lower() in {"1", "true", "yes", "sim", "on"}


def normalizar_float(valor: Any) -> float | None:
    """
    Extrai o primeiro número de um payload.

    Aceita:
      - 23.5
      - "23.5"
      - "23,5"
      - "temperatura=23.5"
      - payloads com unidade, por exemplo "742 mm"
    """
    if valor is None or isinstance(valor, bool):
        return None

    if isinstance(valor, (int, float)):
        return float(valor)

    texto = str(valor).strip()

    if not texto:
        return None

    match = NUMERIC_RE.search(texto)

    if not match:
        return None

    return float(match.group(0).replace(",", "."))


def extrair_primeiro_float(
    dados: dict[str, Any],
    chaves: tuple[str, ...],
) -> float | None:
    """
    Procura valor numérico em um dicionário JSON usando uma lista de chaves.
    """
    for chave in chaves:
        if chave in dados:
            valor = normalizar_float(dados[chave])
            if valor is not None:
                return valor

    return None


def parse_json(payload: str) -> dict[str, Any] | None:
    """
    Tenta interpretar payload como JSON.
    Retorna None se não for JSON válido ou se o JSON não for objeto.
    """
    try:
        dados = json.loads(payload)
    except json.JSONDecodeError:
        return None

    return dados if isinstance(dados, dict) else None


def valor_float_payload(payload: str, json_keys: tuple[str, ...]) -> float:
    """
    Extrai valor float de payload escalar ou JSON.

    O ESP32 atual publica payload escalar, por exemplo:
      "25.30"
      "1013.25"
      "742"

    Esta função também aceita JSON para facilitar testes futuros.
    """
    dados_json = parse_json(payload)

    if dados_json:
        valor = extrair_primeiro_float(dados_json, json_keys)

        if valor is not None:
            return valor

    valor = normalizar_float(payload)

    if valor is None:
        raise ValueError(f"payload sem valor numérico: {payload!r}")

    return valor


def mqtt_connection_succeeded(reason_code: Any) -> bool:
    """
    Compatibilidade com paho-mqtt 1.x e 2.x.

    - Paho 1.x entrega rc inteiro.
    - Paho 2.x pode entregar ReasonCode.
    """
    if reason_code == 0:
        return True

    if hasattr(reason_code, "is_failure"):
        try:
            return not bool(reason_code.is_failure)
        except Exception:
            pass

    return str(reason_code).strip().lower() in {"0", "success"}


def formatar_celula(valor: Any) -> str:
    """
    Formata valores para impressão tabular no terminal.
    """
    if valor is None:
        return "NULL"

    if isinstance(valor, datetime):
        return valor.strftime("%Y-%m-%d %H:%M:%S")

    if isinstance(valor, float):
        return f"{valor:.3f}"

    return str(valor)


# =============================================================================
# CONFIGURAÇÃO
# =============================================================================

@dataclass(frozen=True)
class Config:
    mqtt_brokers: tuple[str, ...]
    mqtt_port: int
    mqtt_keepalive: int
    mqtt_client_id_base: str

    mqtt_topic_prefix_bme688: str
    mqtt_topic_temperatura: str
    mqtt_topic_pressao: str
    mqtt_topic_umidade: str
    mqtt_topic_gas: str
    mqtt_topic_altitude: str
    mqtt_topic_lidar_nearest: str
    mqtt_topic_comando: str

    lidar_zero_as_null: bool

    mysql_host: str
    mysql_database: str
    mysql_user: str
    mysql_password: str | None
    tabela_dados: str

    @classmethod
    def from_env(cls) -> "Config":
        prefixo_bme688 = os.getenv(
            "MQTT_TOPIC_PREFIX",
            "puc/iot/bme688",
        ).strip("/")

        brokers = tuple(
            broker.strip()
            for broker in os.getenv(
                "MQTT_BROKERS",
                "test.mosquitto.org",
            ).split(",")
            if broker.strip()
        )

        if not brokers:
            raise ValueError("MQTT_BROKERS não pode ficar vazio.")

        topic_lidar = os.getenv(
            "MQTT_TOPIC_LIDAR_NEAREST",
            "puc/iot/ld14p/menor_distancia",
        ).strip("/")

        return cls(
            mqtt_brokers=brokers,
            mqtt_port=int(os.getenv("MQTT_PORT", "1883")),
            mqtt_keepalive=int(os.getenv("MQTT_KEEPALIVE", "60")),
            mqtt_client_id_base=os.getenv(
                "MQTT_CLIENT_ID",
                "aegis_rover_mysql_logger",
            ),
            mqtt_topic_prefix_bme688=prefixo_bme688,
            mqtt_topic_temperatura=os.getenv(
                "MQTT_TOPIC_TEMPERATURA",
                f"{prefixo_bme688}/temperatura",
            ).strip("/"),
            mqtt_topic_pressao=os.getenv(
                "MQTT_TOPIC_PRESSAO",
                f"{prefixo_bme688}/pressao",
            ).strip("/"),
            mqtt_topic_umidade=os.getenv(
                "MQTT_TOPIC_UMIDADE",
                f"{prefixo_bme688}/umidade",
            ).strip("/"),
            mqtt_topic_gas=os.getenv(
                "MQTT_TOPIC_GAS",
                f"{prefixo_bme688}/gas",
            ).strip("/"),
            mqtt_topic_altitude=os.getenv(
                "MQTT_TOPIC_ALTITUDE",
                f"{prefixo_bme688}/altitude",
            ).strip("/"),
            mqtt_topic_lidar_nearest=topic_lidar,
            mqtt_topic_comando=os.getenv(
                "MQTT_TOPIC_COMANDO",
                f"{prefixo_bme688}/comando",
            ).strip("/"),
            lidar_zero_as_null=env_bool("LIDAR_ZERO_AS_NULL", True),
            mysql_host=os.getenv("IOT_MYSQL_HOST", "localhost"),
            mysql_database=validar_identificador_mysql(
                os.getenv("IOT_MYSQL_DATABASE", "IoT"),
                "IOT_MYSQL_DATABASE",
            ),
            mysql_user=os.getenv("IOT_MYSQL_USER", "root"),
            mysql_password=os.getenv("IOT_MYSQL_PASSWORD"),
            tabela_dados=validar_identificador_mysql(
                os.getenv("IOT_MYSQL_TABLE_DADOS", "dados_aegis_rover"),
                "IOT_MYSQL_TABLE_DADOS",
            ),
        )


# =============================================================================
# MODELOS
# =============================================================================

@dataclass(frozen=True)
class SensorTopic:
    topic: str
    coluna: str
    json_keys: tuple[str, ...]
    tipo: str  # "bme688" ou "lidar"


@dataclass(frozen=True)
class MqttEvent:
    broker: str
    topic: str
    payload: str


@dataclass
class AmostraAegis:
    """
    Representa uma linha da tabela única.

    A linha é gravada quando todos os 5 campos do BME688 foram recebidos.
    O LiDAR entra como o último valor conhecido de menor distância.
    """
    temperatura: float | None = None
    pressao: float | None = None
    umidade: float | None = None
    gas: float | None = None
    altitude: float | None = None

    menor_distancia_mm: float | None = None
    menor_distancia_cm: float | None = None
    menor_distancia_m: float | None = None

    def as_dict(self) -> dict[str, float | None]:
        return {
            "temperatura": self.temperatura,
            "pressao": self.pressao,
            "umidade": self.umidade,
            "gas": self.gas,
            "altitude": self.altitude,
            "menor_distancia_mm": self.menor_distancia_mm,
            "menor_distancia_cm": self.menor_distancia_cm,
            "menor_distancia_m": self.menor_distancia_m,
        }


# =============================================================================
# AGREGADOR MQTT
# =============================================================================

class BufferAmostraUnica:
    """
    Agrega tópicos escalares em uma única amostra.

    O ESP32 publica BME688 em cinco tópicos separados a cada ~5 s.
    O LD14P publica menor distância em outro tópico, com frequência maior.

    Estratégia:
      - O LiDAR atualiza apenas o "último valor conhecido".
      - O BME688 forma uma amostra quando chegam os 5 campos.
      - Ao completar os 5 campos BME688, o script grava uma linha única
        contendo BME688 + última menor distância LD14P.
    """

    BME_COLUNAS = {
        "temperatura",
        "pressao",
        "umidade",
        "gas",
        "altitude",
    }

    def __init__(self, *, lidar_zero_as_null: bool) -> None:
        self.lidar_zero_as_null = lidar_zero_as_null
        self.lock = threading.RLock()

        # Estado por broker. Isso evita misturar mensagens de brokers diferentes.
        self.estado_por_broker: dict[str, dict[str, Any]] = {}

    def _estado(self, broker: str) -> dict[str, Any]:
        return self.estado_por_broker.setdefault(
            broker,
            {
                "bme_atual": {},
                "bme_recebidas": set(),
                "lidar_mm": None,
                "lidar_cm": None,
                "lidar_m": None,
            },
        )

    def adicionar(
        self,
        *,
        broker: str,
        topico: SensorTopic,
        valor: float,
    ) -> AmostraAegis | None:
        with self.lock:
            estado = self._estado(broker)

            if topico.tipo == "lidar":
                self._atualizar_lidar(estado, valor)
                return None

            if topico.tipo != "bme688":
                return None

            bme_atual: dict[str, float] = estado["bme_atual"]
            bme_recebidas: set[str] = estado["bme_recebidas"]

            # Se uma coluna do BME688 chegou de novo antes de completar as 5,
            # considera-se que um novo ciclo começou. Isso evita misturar
            # temperatura nova com pressão antiga, por exemplo.
            if topico.coluna in bme_recebidas:
                bme_atual.clear()
                bme_recebidas.clear()

            bme_atual[topico.coluna] = valor
            bme_recebidas.add(topico.coluna)

            if bme_recebidas == self.BME_COLUNAS:
                amostra = AmostraAegis(
                    temperatura=bme_atual.get("temperatura"),
                    pressao=bme_atual.get("pressao"),
                    umidade=bme_atual.get("umidade"),
                    gas=bme_atual.get("gas"),
                    altitude=bme_atual.get("altitude"),
                    menor_distancia_mm=estado["lidar_mm"],
                    menor_distancia_cm=estado["lidar_cm"],
                    menor_distancia_m=estado["lidar_m"],
                )

                bme_atual.clear()
                bme_recebidas.clear()

                return amostra

            return None

    def _atualizar_lidar(self, estado: dict[str, Any], valor_mm: float) -> None:
        if self.lidar_zero_as_null and valor_mm <= 0:
            estado["lidar_mm"] = None
            estado["lidar_cm"] = None
            estado["lidar_m"] = None
            return

        estado["lidar_mm"] = valor_mm
        estado["lidar_cm"] = valor_mm / 10.0
        estado["lidar_m"] = valor_mm / 1000.0

    def limpar(self) -> None:
        with self.lock:
            self.estado_por_broker.clear()


# =============================================================================
# BANCO DE DADOS - TABELA ÚNICA
# =============================================================================

class BancoAegisRover:
    """
    Banco de dados com apenas uma tabela operacional.

    Esta classe não cria tabelas de IA, inferências ou tabela separada do LiDAR.
    """

    def __init__(self, config: Config) -> None:
        self.config = config
        self.conexao: mysql.connector.MySQLConnection | None = None
        self.lock = threading.RLock()
        self._password_cache = config.mysql_password

        self.conectar()
        self.criar_tabela_unica()
        self.garantir_migracoes_schema()

    def conectar(self) -> None:
        with self.lock:
            if self.conexao is not None and self.conexao.is_connected():
                return

            if self._password_cache is None:
                self._password_cache = getpass(
                    f"Senha MySQL para {self.config.mysql_user}@{self.config.mysql_host}: "
                )

            self.conexao = mysql.connector.connect(
                host=self.config.mysql_host,
                user=self.config.mysql_user,
                password=self._password_cache,
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

    def criar_tabela_unica(self) -> None:
        with self._cursor() as cursor:
            cursor.execute(
                f"""
                CREATE TABLE IF NOT EXISTS `{self.config.tabela_dados}` (
                    id INT AUTO_INCREMENT PRIMARY KEY,
                    recebido_em TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,

                    temperatura DOUBLE NULL,
                    pressao DOUBLE NULL,
                    umidade DOUBLE NULL,
                    gas DOUBLE NULL,
                    altitude DOUBLE NULL,

                    menor_distancia_mm DOUBLE NULL,
                    menor_distancia_cm DOUBLE NULL,
                    menor_distancia_m DOUBLE NULL,

                    mqtt_broker VARCHAR(128) NULL,
                    origem VARCHAR(128) NULL,

                    INDEX idx_recebido_em (recebido_em),
                    INDEX idx_mqtt_broker (mqtt_broker)
                ) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4
                """
            )

    def coluna_existe(self, coluna: str) -> bool:
        with self._cursor() as cursor:
            cursor.execute(
                """
                SELECT COUNT(*)
                FROM information_schema.columns
                WHERE table_schema = %s
                  AND table_name = %s
                  AND column_name = %s
                """,
                (self.config.mysql_database, self.config.tabela_dados, coluna),
            )
            resultado = cursor.fetchone()

        return bool(resultado and resultado[0] > 0)

    def indice_existe(self, indice: str) -> bool:
        with self._cursor() as cursor:
            cursor.execute(
                """
                SELECT COUNT(*)
                FROM information_schema.statistics
                WHERE table_schema = %s
                  AND table_name = %s
                  AND index_name = %s
                """,
                (self.config.mysql_database, self.config.tabela_dados, indice),
            )
            resultado = cursor.fetchone()

        return bool(resultado and resultado[0] > 0)

    def garantir_coluna(self, coluna: str, definicao_sql: str) -> None:
        if self.coluna_existe(coluna):
            return

        with self._cursor() as cursor:
            cursor.execute(
                f"""
                ALTER TABLE `{self.config.tabela_dados}`
                ADD COLUMN `{coluna}` {definicao_sql}
                """
            )

        print(f"[BD] Coluna adicionada: {self.config.tabela_dados}.{coluna}")

    def garantir_indice(self, indice: str, definicao_sql: str) -> None:
        if self.indice_existe(indice):
            return

        try:
            with self._cursor() as cursor:
                cursor.execute(
                    f"""
                    ALTER TABLE `{self.config.tabela_dados}`
                    ADD INDEX `{indice}` {definicao_sql}
                    """
                )

            print(f"[BD] Índice adicionado: {self.config.tabela_dados}.{indice}")

        except MySQLError as exc:
            print(f"[BD] Não foi possível criar índice {indice}: {exc}")

    def garantir_migracoes_schema(self) -> None:
        """
        Garante que a tabela tenha as colunas novas.

        Observação:
          Esta rotina não remove colunas antigas automaticamente. Para uma
          tabela limpa apenas com o schema atual, use o comando CLI:
            recriar tabela
        """
        for coluna in (
            "temperatura",
            "pressao",
            "umidade",
            "gas",
            "altitude",
            "menor_distancia_mm",
            "menor_distancia_cm",
            "menor_distancia_m",
        ):
            self.garantir_coluna(coluna, "DOUBLE NULL")

        self.garantir_coluna("mqtt_broker", "VARCHAR(128) NULL")
        self.garantir_coluna("origem", "VARCHAR(128) NULL")

        self.garantir_indice("idx_recebido_em", "(`recebido_em`)")
        self.garantir_indice("idx_mqtt_broker", "(`mqtt_broker`)")

    def recriar_tabela_unica(self) -> None:
        """
        Remove e recria a tabela única.

        Uso recomendado quando há uma tabela antiga com colunas em inglês
        e você quer ficar apenas com as colunas novas em português.
        """
        with self._cursor() as cursor:
            cursor.execute(f"DROP TABLE IF EXISTS `{self.config.tabela_dados}`")

        self.criar_tabela_unica()
        print(f"[BD] Tabela `{self.config.tabela_dados}` recriada com schema único.")

    def inserir_amostra(
        self,
        amostra: AmostraAegis,
        *,
        mqtt_broker: str | None,
        origem: str,
    ) -> int:
        dados = amostra.as_dict()
        dados["mqtt_broker"] = mqtt_broker
        dados["origem"] = origem

        colunas = ", ".join(f"`{coluna}`" for coluna in dados)
        placeholders = ", ".join(["%s"] * len(dados))
        valores = tuple(dados.values())

        with self._cursor() as cursor:
            cursor.execute(
                f"""
                INSERT INTO `{self.config.tabela_dados}`
                ({colunas})
                VALUES ({placeholders})
                """,
                valores,
            )
            return int(cursor.lastrowid)

    def processar_amostra(
        self,
        amostra: AmostraAegis,
        *,
        origem: str,
        mqtt_broker: str | None,
    ) -> None:
        try:
            row_id = self.inserir_amostra(
                amostra,
                mqtt_broker=mqtt_broker,
                origem=origem,
            )

            print(
                f"[BD] id={row_id} | "
                f"T={amostra.temperatura} | "
                f"P={amostra.pressao} | "
                f"H={amostra.umidade} | "
                f"G={amostra.gas} | "
                f"A={amostra.altitude} | "
                f"dist={amostra.menor_distancia_cm} cm / {amostra.menor_distancia_m} m | "
                f"broker={mqtt_broker or '-'}"
            )

        except Exception as exc:
            print(f"[ERRO] Falha ao gravar amostra: {exc}")

    def exibir(self, limite: int = 10) -> None:
        query = (
            f"""
            SELECT
                id,
                recebido_em,
                temperatura,
                pressao,
                umidade,
                gas,
                altitude,
                menor_distancia_mm,
                menor_distancia_cm,
                menor_distancia_m,
                mqtt_broker,
                origem
            FROM `{self.config.tabela_dados}`
            ORDER BY id DESC
            LIMIT %s
            """
        )

        self._imprimir_tabela(
            titulo=f"{self.config.mysql_database}.{self.config.tabela_dados}",
            query=query,
            params=(limite,),
            headers=(
                "ID",
                "TIMESTAMP",
                "TEMP",
                "PRESSAO",
                "UMIDADE",
                "GAS",
                "ALTITUDE",
                "DIST_MM",
                "DIST_CM",
                "DIST_M",
                "BROKER",
                "ORIGEM",
            ),
        )

    def apagar_tabela(self) -> None:
        with self._cursor() as cursor:
            cursor.execute(f"TRUNCATE TABLE `{self.config.tabela_dados}`")

        print(f"[BD] Tabela `{self.config.tabela_dados}` apagada.")

    def fechar(self) -> None:
        with self.lock:
            if self.conexao is not None and self.conexao.is_connected():
                self.conexao.close()
                print("[BD] Conexão encerrada.")

    def _imprimir_tabela(
        self,
        *,
        titulo: str,
        query: str,
        params: tuple[Any, ...],
        headers: tuple[str, ...],
    ) -> None:
        with self._cursor() as cursor:
            cursor.execute(query, params)
            linhas = cursor.fetchall()

        print("\n" + "=" * 150)
        print(titulo)
        print("=" * 150)
        print(" | ".join(headers))
        print("-" * 150)

        for linha in linhas:
            print(" | ".join(formatar_celula(valor) for valor in linha))

        print("=" * 150 + "\n")

    def _cursor(self, dictionary: bool = False) -> "CursorTransacional":
        return CursorTransacional(self, dictionary=dictionary)


class CursorTransacional:
    """
    Context manager para cursor MySQL com commit/rollback automático.
    """

    def __init__(self, banco: BancoAegisRover, dictionary: bool = False):
        self.banco = banco
        self.dictionary = dictionary
        self.cursor = None

    def __enter__(self):
        self.banco.lock.acquire()
        self.banco.conectar()
        assert self.banco.conexao is not None
        self.cursor = self.banco.conexao.cursor(dictionary=self.dictionary)
        return self.cursor

    def __exit__(self, exc_type, exc, tb):
        assert self.banco.conexao is not None

        if exc_type is None:
            self.banco.conexao.commit()
        else:
            self.banco.conexao.rollback()

        if self.cursor is not None:
            self.cursor.close()

        self.banco.lock.release()
        return False


# =============================================================================
# MQTT
# =============================================================================

def criar_topicos_sensores(config: Config) -> dict[str, SensorTopic]:
    """
    Mapa exato dos tópicos publicados pelo ESP32.
    """
    return {
        config.mqtt_topic_temperatura: SensorTopic(
            topic=config.mqtt_topic_temperatura,
            coluna="temperatura",
            json_keys=("temperatura", "temperature", "temperature_c", "valor", "value"),
            tipo="bme688",
        ),
        config.mqtt_topic_pressao: SensorTopic(
            topic=config.mqtt_topic_pressao,
            coluna="pressao",
            json_keys=("pressao", "pressure", "pressure_hpa", "valor", "value"),
            tipo="bme688",
        ),
        config.mqtt_topic_umidade: SensorTopic(
            topic=config.mqtt_topic_umidade,
            coluna="umidade",
            json_keys=("umidade", "humidity", "humidity_pct", "relative_humidity", "valor", "value"),
            tipo="bme688",
        ),
        config.mqtt_topic_gas: SensorTopic(
            topic=config.mqtt_topic_gas,
            coluna="gas",
            json_keys=("gas", "gas_resistance", "gas_kohm", "gas_kohms", "resistance_gassensor", "valor", "value"),
            tipo="bme688",
        ),
        config.mqtt_topic_altitude: SensorTopic(
            topic=config.mqtt_topic_altitude,
            coluna="altitude",
            json_keys=("altitude", "altitude_m", "valor", "value"),
            tipo="bme688",
        ),
        config.mqtt_topic_lidar_nearest: SensorTopic(
            topic=config.mqtt_topic_lidar_nearest,
            coluna="menor_distancia_mm",
            json_keys=("menor_distancia", "menor_distancia_mm", "distance_mm", "valor", "value"),
            tipo="lidar",
        ),
    }


class ColetorMQTTAegis:
    """
    Cliente MQTT que recebe os tópicos do ESP32 e envia amostras ao banco.
    """

    def __init__(self, config: Config, banco: BancoAegisRover) -> None:
        self.config = config
        self.banco = banco

        self.topicos_sensores = criar_topicos_sensores(config)

        self.agregador = BufferAmostraUnica(
            lidar_zero_as_null=config.lidar_zero_as_null,
        )

        self.fila: queue.Queue[MqttEvent | None] = queue.Queue()
        self.stop_event = threading.Event()
        self.clients: list[mqtt.Client] = []

        self.thread_processador = threading.Thread(
            target=self._processar_fila,
            daemon=True,
        )

    def conectar(self) -> None:
        self.thread_processador.start()

        for broker in self.config.mqtt_brokers:
            client = self._criar_cliente_mqtt(broker)

            print(f"[MQTT][{broker}] Conectando em {broker}:{self.config.mqtt_port}")

            client.connect_async(
                broker,
                self.config.mqtt_port,
                self.config.mqtt_keepalive,
            )
            client.loop_start()
            self.clients.append(client)

    def encerrar(self) -> None:
        self.stop_event.set()
        self.fila.put(None)

        for client in self.clients:
            try:
                client.loop_stop()
                client.disconnect()
            except Exception:
                pass

        if self.thread_processador.is_alive():
            self.thread_processador.join(timeout=2.0)

    def publicar_comando_esp32(self, comando: str) -> None:
        """
        Publica comandos no tópico escutado pelo ESP32:
          puc/iot/bme688/comando

        Comandos reconhecidos pelo firmware:
          delete
          termina
        """
        comando = comando.strip()

        if not comando:
            print("[MQTT-CMD] Comando vazio ignorado.")
            return

        if not self.clients:
            print("[MQTT-CMD] Nenhum cliente MQTT ativo.")
            return

        for client in self.clients:
            result = client.publish(self.config.mqtt_topic_comando, comando)
            rc = getattr(result, "rc", None)

            print(
                f"[MQTT-CMD] Publicado em {self.config.mqtt_topic_comando}: "
                f"{comando!r} | rc={rc}"
            )

    def _criar_cliente_mqtt(self, broker: str) -> mqtt.Client:
        broker_sanitizado = re.sub(r"[^A-Za-z0-9_]", "_", broker)
        client_id = (
            f"{self.config.mqtt_client_id_base}_"
            f"{broker_sanitizado}_"
            f"{uuid.uuid4().hex[:8]}"
        )

        # Compatível com paho-mqtt 2.x e fallback para 1.x.
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
        reason_code: Any,
        properties: Any = None,
    ) -> None:
        broker = userdata["broker"]

        if not mqtt_connection_succeeded(reason_code):
            print(f"[MQTT][{broker}] Falha de conexão: {reason_code}")
            return

        for item in self.topicos_sensores.values():
            client.subscribe(item.topic)

        client.subscribe(self.config.mqtt_topic_comando)

        print(f"[MQTT][{broker}] Conectado.")
        print("[MQTT] Tópicos assinados:")
        for item in self.topicos_sensores.values():
            print(f"  - {item.topic}")
        print(f"  - {self.config.mqtt_topic_comando}  [comando ESP32]")

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
                self._processar_mensagem(evento)
            except Exception as exc:
                print(
                    f"[ERRO] Falha ao processar {evento.topic} "
                    f"via {evento.broker}: {exc}"
                )
            finally:
                self.fila.task_done()

    def _processar_mensagem(self, evento: MqttEvent) -> None:
        topico = evento.topic
        payload = evento.payload

        if topico == self.config.mqtt_topic_comando:
            print(f"[MQTT-CMD][{evento.broker}] eco/comando recebido: {payload}")
            return

        topico_info = self.topicos_sensores.get(topico)

        if not topico_info:
            return

        valor = valor_float_payload(payload, topico_info.json_keys)

        if topico_info.tipo == "lidar":
            cm = valor / 10.0
            m = valor / 1000.0

            if self.config.lidar_zero_as_null and valor <= 0:
                print(f"[MQTT][{evento.broker}] LD14P sem detecção válida.")
            else:
                print(
                    f"[MQTT][{evento.broker}] "
                    f"menor_distancia={valor:.0f} mm | {cm:.1f} cm | {m:.3f} m"
                )
        else:
            print(f"[MQTT][{evento.broker}] {topico_info.coluna}={valor:.3f}")

        amostra = self.agregador.adicionar(
            broker=evento.broker,
            topico=topico_info,
            valor=valor,
        )

        if amostra:
            self.banco.processar_amostra(
                amostra,
                origem="mqtt_scalar:esp32_aegis",
                mqtt_broker=evento.broker,
            )


# =============================================================================
# CLI
# =============================================================================

def confirmar_operacao(frase: str) -> bool:
    print(f"[CONFIRMAÇÃO] Operação destrutiva solicitada: {frase}")
    resposta = input(f"Digite exatamente '{frase}' para confirmar: ").strip()
    return resposta == frase


def imprimir_ajuda() -> None:
    print(
        """
Comandos:
  ver [n]                    Exibe as últimas n linhas da tabela única
  config                     Mostra a configuração ativa
  migrar                     Executa novamente as migrações de schema

  mqtt delete                Publica 'delete' para o ESP32
  mqtt termina               Publica 'termina' para o ESP32
  mqtt comando <texto>       Publica comando personalizado para o ESP32

  apagar tabela              TRUNCATE na tabela única, com confirmação
  recriar tabela             DROP + CREATE da tabela única, com confirmação
                             Use para remover colunas antigas e ficar só com:
                               temperatura, pressao, umidade, gas, altitude,
                               menor_distancia_mm, menor_distancia_cm,
                               menor_distancia_m

  ajuda                      Mostra este painel
  sair                       Encerra o programa
"""
    )


def imprimir_config(config: Config) -> None:
    print("\n" + "=" * 90)
    print("CONFIGURAÇÃO ATIVA")
    print("=" * 90)
    print(f"MQTT brokers: {', '.join(config.mqtt_brokers)}")
    print(f"MQTT porta: {config.mqtt_port}")
    print(f"MQTT BME688 prefixo: {config.mqtt_topic_prefix_bme688}")
    print(f"MQTT temperatura: {config.mqtt_topic_temperatura}")
    print(f"MQTT pressão: {config.mqtt_topic_pressao}")
    print(f"MQTT umidade: {config.mqtt_topic_umidade}")
    print(f"MQTT gás: {config.mqtt_topic_gas}")
    print(f"MQTT altitude: {config.mqtt_topic_altitude}")
    print(f"MQTT LD14P menor distância: {config.mqtt_topic_lidar_nearest}")
    print(f"MQTT comando ESP32: {config.mqtt_topic_comando}")
    print(f"LIDAR_ZERO_AS_NULL: {config.lidar_zero_as_null}")
    print(f"MySQL host: {config.mysql_host}")
    print(f"MySQL database: {config.mysql_database}")
    print(f"MySQL tabela única: {config.tabela_dados}")
    print("=" * 90 + "\n")


def executar_comando_cli(
    comando: str,
    banco: BancoAegisRover,
    coletor: ColetorMQTTAegis,
    config: Config,
) -> bool:
    comando = comando.strip()
    comando_lower = comando.lower()

    if not comando:
        return True

    if comando_lower in {"sair", "exit", "quit"}:
        return False

    if comando_lower in {"ajuda", "help", "h", "menu"}:
        imprimir_ajuda()
        return True

    if comando_lower in {"config", "conf"}:
        imprimir_config(config)
        return True

    if comando_lower == "migrar":
        banco.garantir_migracoes_schema()
        print("[BD] Migrações verificadas/executadas.")
        return True

    if comando_lower.startswith("ver"):
        partes = comando.split(maxsplit=1)
        limite = 10

        if len(partes) == 2:
            try:
                limite = max(1, int(partes[1]))
            except ValueError:
                print("[APP] Uso: ver [n]")
                return True

        banco.exibir(limite=limite)
        return True

    if comando_lower == "mqtt delete":
        coletor.publicar_comando_esp32("delete")
        return True

    if comando_lower == "mqtt termina":
        coletor.publicar_comando_esp32("termina")
        return True

    if comando_lower.startswith("mqtt comando "):
        comando_mqtt = comando.split(maxsplit=2)[2]
        coletor.publicar_comando_esp32(comando_mqtt)
        return True

    if comando_lower == "apagar tabela":
        frase = f"APAGAR {banco.config.tabela_dados}"
        if confirmar_operacao(frase):
            banco.apagar_tabela()
        else:
            print("[BD] Operação cancelada.")
        return True

    if comando_lower == "recriar tabela":
        frase = f"RECRIAR {banco.config.tabela_dados}"
        if confirmar_operacao(frase):
            banco.recriar_tabela_unica()
        else:
            print("[BD] Operação cancelada.")
        return True

    print(f"[APP] Comando não reconhecido: {comando!r}. Digite 'ajuda'.")
    return True


# =============================================================================
# MAIN
# =============================================================================

def main() -> None:
    config = Config.from_env()
    banco = BancoAegisRover(config)
    coletor = ColetorMQTTAegis(config, banco)

    print("\nAegisRover ESP32 BME688 + LD14P MQTT -> MySQL")
    print(f"Banco: {config.mysql_database}")
    print(f"Tabela única: {config.tabela_dados}")

    imprimir_config(config)
    imprimir_ajuda()

    try:
        coletor.conectar()

        while True:
            comando = input("> ").strip()

            if not executar_comando_cli(comando, banco, coletor, config):
                break

    except KeyboardInterrupt:
        print("\n[APP] Finalizado via Ctrl+C.")

    finally:
        coletor.encerrar()
        banco.fechar()
        print("[APP] Sistema encerrado.")


if __name__ == "__main__":
    main()
