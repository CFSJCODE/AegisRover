from __future__ import annotations

import json
import os
import queue
import re
import threading
import time
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

CLASSES_VALIDAS = {
    "ar_limpo",
    "alcool",
    "fumaca",
    "cafe",
    "ambiente_externo",
}

FASES_VALIDAS = {
    "baseline",
    "exposicao",
    "recuperacao",
    "ambiente",
    "indefinida",
}

ORIGENS_MODELO_VALIDAS = {
    "python",
    "bsec",
    "bsec2",
    "embarcado",
    "manual",
}


# =============================================================================
# UTILITÁRIOS
# =============================================================================

def validar_identificador_mysql(valor: str, nome: str) -> str:
    if not re.fullmatch(r"[A-Za-z0-9_]+", valor):
        raise ValueError(
            f"{nome} deve conter apenas letras, números e underscore: {valor!r}"
        )
    return valor


def normalizar_float(valor: Any) -> float | None:
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
    for chave in chaves:
        if chave in dados:
            valor = normalizar_float(dados[chave])
            if valor is not None:
                return valor
    return None


def parse_json(payload: str) -> dict[str, Any] | None:
    try:
        dados = json.loads(payload)
    except json.JSONDecodeError:
        return None

    return dados if isinstance(dados, dict) else None


def texto_ou_none(valor: Any) -> str | None:
    if valor is None:
        return None

    texto = str(valor).strip()
    return texto or None


def mqtt_connection_succeeded(reason_code: Any) -> bool:
    if reason_code == 0:
        return True

    return str(reason_code).strip().lower() in {"0", "success"}


def formatar_celula(valor: Any) -> str:
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
    mqtt_topic_prefix: str
    mqtt_topic_json: str
    mqtt_topic_aegis_json: str

    mysql_host: str
    mysql_database: str
    mysql_user: str
    mysql_password: str | None
    tabela_visualizacao: str
    tabela_ia: str
    tabela_inferencias: str

    @classmethod
    def from_env(cls) -> "Config":
        prefixo = os.getenv(
            "MQTT_TOPIC_PREFIX",
            "cfsj/aegis/bme688",
        ).strip("/")

        brokers = tuple(
            broker.strip()
            for broker in os.getenv(
                "MQTT_BROKERS",
                "broker.emqx.io,test.mosquitto.org",
            ).split(",")
            if broker.strip()
        )

        return cls(
            mqtt_brokers=brokers,
            mqtt_port=int(os.getenv("MQTT_PORT", "1883")),
            mqtt_keepalive=int(os.getenv("MQTT_KEEPALIVE", "60")),
            mqtt_client_id_base=os.getenv(
                "MQTT_CLIENT_ID",
                "cfsj_iot_bme688_bd_ia",
            ),
            mqtt_topic_prefix=prefixo,
            mqtt_topic_json=os.getenv(
                "MQTT_TOPIC_JSON",
                f"{prefixo}/telemetry",
            ).strip("/"),
            mqtt_topic_aegis_json=os.getenv(
                "MQTT_TOPIC_AEGIS_JSON",
                "aegis/sensor/bme688/sample",
            ).strip("/"),
            mysql_host=os.getenv("IOT_MYSQL_HOST", "localhost"),
            mysql_database=validar_identificador_mysql(
                os.getenv("IOT_MYSQL_DATABASE", "IoT"),
                "IOT_MYSQL_DATABASE",
            ),
            mysql_user=os.getenv("IOT_MYSQL_USER", "root"),
            mysql_password=os.getenv("IOT_MYSQL_PASSWORD"),
            tabela_visualizacao=validar_identificador_mysql(
                os.getenv("IOT_MYSQL_TABLE_VISUALIZACAO", "dados_bme688_colunar"),
                "IOT_MYSQL_TABLE_VISUALIZACAO",
            ),
            tabela_ia=validar_identificador_mysql(
                os.getenv("IOT_MYSQL_TABLE_IA", "dados_bme688_ia_treinamento"),
                "IOT_MYSQL_TABLE_IA",
            ),
            tabela_inferencias=validar_identificador_mysql(
                os.getenv("IOT_MYSQL_TABLE_INFERENCIAS", "dados_bme688_ia_inferencias"),
                "IOT_MYSQL_TABLE_INFERENCIAS",
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


@dataclass(frozen=True)
class MqttEvent:
    broker: str
    topic: str
    payload: str


@dataclass
class AmostraBME688:
    temperatura: float | None = None
    pressao: float | None = None
    umidade: float | None = None
    gas: float | None = None
    altitude: float | None = None

    bsec_iaq: float | None = None
    bsec_static_iaq: float | None = None
    bsec_co2_equivalent: float | None = None
    bsec_breath_voc_equivalent: float | None = None
    bsec_gas_percentage: float | None = None
    bsec_compensated_temperature: float | None = None
    bsec_compensated_humidity: float | None = None

    heater_profile: str | None = None
    duty_cycle: str | None = None
    bsec_config_nome: str | None = None
    bme_ai_studio_project: str | None = None

    @classmethod
    def from_json(cls, dados: dict[str, Any]) -> "AmostraBME688":
        return cls(
            temperatura=extrair_primeiro_float(
                dados,
                ("temperatura", "temperature", "temperature_c"),
            ),
            pressao=extrair_primeiro_float(
                dados,
                ("pressao", "pressure", "pressure_hpa"),
            ),
            umidade=extrair_primeiro_float(
                dados,
                ("umidade", "humidity", "humidity_pct"),
            ),
            gas=extrair_primeiro_float(
                dados,
                ("gas", "gas_resistance", "gas_kohm", "gas_kohms"),
            ),
            altitude=extrair_primeiro_float(
                dados,
                ("altitude", "altitude_m"),
            ),
            bsec_iaq=extrair_primeiro_float(
                dados,
                ("bsec_iaq", "iaq"),
            ),
            bsec_static_iaq=extrair_primeiro_float(
                dados,
                ("bsec_static_iaq", "static_iaq"),
            ),
            bsec_co2_equivalent=extrair_primeiro_float(
                dados,
                ("bsec_co2_equivalent", "co2_equivalent"),
            ),
            bsec_breath_voc_equivalent=extrair_primeiro_float(
                dados,
                ("bsec_breath_voc_equivalent", "breath_voc_equivalent"),
            ),
            bsec_gas_percentage=extrair_primeiro_float(
                dados,
                ("bsec_gas_percentage", "gas_percentage"),
            ),
            bsec_compensated_temperature=extrair_primeiro_float(
                dados,
                ("bsec_compensated_temperature", "compensated_temperature"),
            ),
            bsec_compensated_humidity=extrair_primeiro_float(
                dados,
                ("bsec_compensated_humidity", "compensated_humidity"),
            ),
            heater_profile=texto_ou_none(dados.get("heater_profile")),
            duty_cycle=texto_ou_none(dados.get("duty_cycle")),
            bsec_config_nome=texto_ou_none(
                dados.get("bsec_config_nome") or dados.get("bsec_config_name")
            ),
            bme_ai_studio_project=texto_ou_none(
                dados.get("bme_ai_studio_project") or dados.get("bme_project")
            ),
        )

    @classmethod
    def from_visualizacao_dict(cls, dados: dict[str, float]) -> "AmostraBME688":
        return cls(
            temperatura=dados.get("temperatura"),
            pressao=dados.get("pressao"),
            umidade=dados.get("umidade"),
            gas=dados.get("gas"),
            altitude=dados.get("altitude"),
        )

    def visualizacao_dict(self) -> dict[str, float | None]:
        return {
            "temperatura": self.temperatura,
            "pressao": self.pressao,
            "umidade": self.umidade,
            "gas": self.gas,
            "altitude": self.altitude,
        }


# =============================================================================
# BUFFER AGREGADOR
# =============================================================================

class BufferAgregador:
    """
    Reagrupa leituras publicadas em tópicos separados.

    Tópicos do firmware:
    - cfsj/aegis/bme688/temperature
    - cfsj/aegis/bme688/pressure
    - cfsj/aegis/bme688/humidity
    - cfsj/aegis/bme688/gas
    - cfsj/aegis/bme688/altitude
    """

    def __init__(self, colunas_esperadas: int) -> None:
        self.colunas_esperadas = colunas_esperadas
        self.buffers_por_broker: dict[str, dict[str, float]] = {}
        self.lock = threading.RLock()

    def adicionar_leitura(
        self,
        broker: str,
        coluna: str,
        valor: float,
    ) -> dict[str, float] | None:
        with self.lock:
            buffer_atual = self.buffers_por_broker.setdefault(broker, {})

            if coluna in buffer_atual:
                buffer_atual.clear()
                buffer_atual[coluna] = valor
                return None

            buffer_atual[coluna] = valor

            if len(buffer_atual) == self.colunas_esperadas:
                dados_completos = buffer_atual.copy()
                buffer_atual.clear()
                return dados_completos

            return None

    def limpar(self) -> None:
        with self.lock:
            self.buffers_por_broker.clear()


# =============================================================================
# SESSÃO IA
# =============================================================================

@dataclass
class SessaoColetaIA:
    sessao_id: str | None = None
    classe: str | None = None
    fase: str = "indefinida"
    origem: str | None = None
    operador: str | None = None
    observacao: str | None = None
    ativa: bool = False
    amostras_coletadas: int = 0

    def iniciar(self, sessao_id: str | None = None) -> None:
        self.sessao_id = sessao_id or datetime.now().strftime("S%Y%m%d_%H%M%S")
        self.ativa = True
        self.amostras_coletadas = 0

    def parar(self) -> None:
        self.ativa = False

    def definir_classe(self, classe: str) -> None:
        classe = classe.strip().lower()

        if classe not in CLASSES_VALIDAS:
            raise ValueError(
                f"classe inválida: {classe}. "
                f"Use: {', '.join(sorted(CLASSES_VALIDAS))}"
            )

        self.classe = classe

    def definir_fase(self, fase: str) -> None:
        fase = fase.strip().lower()

        if fase not in FASES_VALIDAS:
            raise ValueError(
                f"fase inválida: {fase}. "
                f"Use: {', '.join(sorted(FASES_VALIDAS))}"
            )

        self.fase = fase

    def pronta_para_gravar(self) -> bool:
        return self.ativa and bool(self.sessao_id) and bool(self.classe)

    def status(self) -> str:
        return (
            f"sessao_id={self.sessao_id or '-'} | "
            f"ativa={self.ativa} | "
            f"classe={self.classe or '-'} | "
            f"fase={self.fase} | "
            f"origem={self.origem or '-'} | "
            f"operador={self.operador or '-'} | "
            f"amostras={self.amostras_coletadas} | "
            f"obs={self.observacao or '-'}"
        )


# =============================================================================
# BANCO DE DADOS
# =============================================================================

class BancoBME688IA:
    def __init__(self, config: Config, sessao: SessaoColetaIA) -> None:
        self.config = config
        self.sessao = sessao
        self.conexao: mysql.connector.MySQLConnection | None = None
        self.lock = threading.RLock()
        self._password_cache = config.mysql_password

        self.conectar()
        self.criar_tabelas()

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

    def criar_tabelas(self) -> None:
        self.criar_tabela_visualizacao()
        self.criar_tabela_ia_treinamento()
        self.criar_tabela_inferencias()
        self.garantir_migracoes_schema()

    def criar_tabela_visualizacao(self) -> None:
        with self._cursor() as cursor:
            cursor.execute(
                f"""
                CREATE TABLE IF NOT EXISTS `{self.config.tabela_visualizacao}` (
                    id INT AUTO_INCREMENT PRIMARY KEY,
                    recebido_em TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
                    temperatura DOUBLE NULL,
                    pressao DOUBLE NULL,
                    umidade DOUBLE NULL,
                    gas DOUBLE NULL,
                    altitude DOUBLE NULL,
                    mqtt_broker VARCHAR(128) NULL,
                    origem VARCHAR(128) NULL,
                    INDEX idx_visual_recebido_em (recebido_em),
                    INDEX idx_visual_broker (mqtt_broker),
                    INDEX idx_visual_origem (origem)
                ) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4
                """
            )

    def criar_tabela_ia_treinamento(self) -> None:
        query = f"""
            CREATE TABLE IF NOT EXISTS `{self.config.tabela_ia}` (
                id INT AUTO_INCREMENT PRIMARY KEY,
                recebido_em TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
                sessao_id VARCHAR(64) NOT NULL,
                classe VARCHAR(64) NOT NULL,
                fase VARCHAR(32) NOT NULL DEFAULT 'indefinida',
                origem VARCHAR(128) NULL,
                operador VARCHAR(96) NULL,
                observacao TEXT NULL,
                temperatura DOUBLE NULL,
                pressao DOUBLE NULL,
                umidade DOUBLE NULL,
                gas DOUBLE NULL,
                altitude DOUBLE NULL,
                bsec_iaq DOUBLE NULL,
                bsec_static_iaq DOUBLE NULL,
                bsec_co2_equivalent DOUBLE NULL,
                bsec_breath_voc_equivalent DOUBLE NULL,
                bsec_gas_percentage DOUBLE NULL,
                bsec_compensated_temperature DOUBLE NULL,
                bsec_compensated_humidity DOUBLE NULL,
                heater_profile VARCHAR(96) NULL,
                duty_cycle VARCHAR(96) NULL,
                bsec_config_nome VARCHAR(128) NULL,
                bme_ai_studio_project VARCHAR(128) NULL,
                mqtt_broker VARCHAR(128) NULL,
                amostra_valida BOOLEAN NOT NULL DEFAULT TRUE,
                motivo_invalidacao VARCHAR(255) NULL,
                INDEX idx_ia_sessao (sessao_id),
                INDEX idx_ia_classe (classe),
                INDEX idx_ia_fase (fase),
                INDEX idx_ia_recebido_em (recebido_em),
                INDEX idx_ia_broker (mqtt_broker)
            ) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4
        """
        with self._cursor() as cursor:
            cursor.execute(query)

    def criar_tabela_inferencias(self) -> None:
        query = f"""
            CREATE TABLE IF NOT EXISTS `{self.config.tabela_inferencias}` (
                id INT AUTO_INCREMENT PRIMARY KEY,
                inferido_em TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
                treinamento_id INT NULL,
                sessao_id VARCHAR(64) NULL,
                modelo_nome VARCHAR(128) NOT NULL,
                modelo_versao VARCHAR(64) NULL,
                origem_modelo VARCHAR(32) NOT NULL DEFAULT 'python',
                classe_predita VARCHAR(64) NOT NULL,
                classe_real VARCHAR(64) NULL,
                confianca DOUBLE NULL,
                prob_ar_limpo DOUBLE NULL,
                prob_alcool DOUBLE NULL,
                prob_fumaca DOUBLE NULL,
                prob_cafe DOUBLE NULL,
                prob_ambiente_externo DOUBLE NULL,
                prob_outros DOUBLE NULL,
                temperatura DOUBLE NULL,
                pressao DOUBLE NULL,
                umidade DOUBLE NULL,
                gas DOUBLE NULL,
                altitude DOUBLE NULL,
                payload_json TEXT NULL,
                observacao TEXT NULL,
                INDEX idx_inf_modelo (modelo_nome),
                INDEX idx_inf_classe_predita (classe_predita),
                INDEX idx_inf_inferido_em (inferido_em),
                INDEX idx_inf_treinamento_id (treinamento_id)
            ) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4
        """
        with self._cursor() as cursor:
            cursor.execute(query)

    # -------------------------------------------------------------------------
    # MIGRAÇÕES AUTOMÁTICAS
    # -------------------------------------------------------------------------

    def coluna_existe(self, tabela: str, coluna: str) -> bool:
        with self._cursor() as cursor:
            cursor.execute(
                """
                SELECT COUNT(*)
                FROM information_schema.columns
                WHERE table_schema = %s
                  AND table_name = %s
                  AND column_name = %s
                """,
                (self.config.mysql_database, tabela, coluna),
            )
            resultado = cursor.fetchone()

        return bool(resultado and resultado[0] > 0)

    def indice_existe(self, tabela: str, indice: str) -> bool:
        with self._cursor() as cursor:
            cursor.execute(
                """
                SELECT COUNT(*)
                FROM information_schema.statistics
                WHERE table_schema = %s
                  AND table_name = %s
                  AND index_name = %s
                """,
                (self.config.mysql_database, tabela, indice),
            )
            resultado = cursor.fetchone()

        return bool(resultado and resultado[0] > 0)

    def garantir_coluna(self, tabela: str, coluna: str, definicao_sql: str) -> None:
        if self.coluna_existe(tabela, coluna):
            return

        with self._cursor() as cursor:
            cursor.execute(
                f"""
                ALTER TABLE `{tabela}`
                ADD COLUMN `{coluna}` {definicao_sql}
                """
            )

        print(f"[BD] Coluna adicionada automaticamente: {tabela}.{coluna}")

    def garantir_indice(self, tabela: str, indice: str, definicao_sql: str) -> None:
        if self.indice_existe(tabela, indice):
            return

        try:
            with self._cursor() as cursor:
                cursor.execute(
                    f"""
                    ALTER TABLE `{tabela}`
                    ADD INDEX `{indice}` {definicao_sql}
                    """
                )

            print(f"[BD] Índice adicionado automaticamente: {tabela}.{indice}")

        except MySQLError as exc:
            print(f"[BD] Não foi possível criar índice {tabela}.{indice}: {exc}")

    def garantir_migracoes_schema(self) -> None:
        # Tabela de visualização
        for coluna in ("temperatura", "pressao", "umidade", "gas", "altitude"):
            self.garantir_coluna(self.config.tabela_visualizacao, coluna, "DOUBLE NULL")

        self.garantir_coluna(
            self.config.tabela_visualizacao,
            "mqtt_broker",
            "VARCHAR(128) NULL",
        )
        self.garantir_coluna(
            self.config.tabela_visualizacao,
            "origem",
            "VARCHAR(128) NULL",
        )

        self.garantir_indice(
            self.config.tabela_visualizacao,
            "idx_visual_recebido_em",
            "(`recebido_em`)",
        )
        self.garantir_indice(
            self.config.tabela_visualizacao,
            "idx_visual_broker",
            "(`mqtt_broker`)",
        )
        self.garantir_indice(
            self.config.tabela_visualizacao,
            "idx_visual_origem",
            "(`origem`)",
        )

        # Tabela de treinamento IA
        self.garantir_coluna(self.config.tabela_ia, "sessao_id", "VARCHAR(64) NULL")
        self.garantir_coluna(self.config.tabela_ia, "classe", "VARCHAR(64) NULL")
        self.garantir_coluna(
            self.config.tabela_ia,
            "fase",
            "VARCHAR(32) NOT NULL DEFAULT 'indefinida'",
        )
        self.garantir_coluna(self.config.tabela_ia, "origem", "VARCHAR(128) NULL")
        self.garantir_coluna(self.config.tabela_ia, "operador", "VARCHAR(96) NULL")
        self.garantir_coluna(self.config.tabela_ia, "observacao", "TEXT NULL")

        for coluna in (
            "temperatura",
            "pressao",
            "umidade",
            "gas",
            "altitude",
            "bsec_iaq",
            "bsec_static_iaq",
            "bsec_co2_equivalent",
            "bsec_breath_voc_equivalent",
            "bsec_gas_percentage",
            "bsec_compensated_temperature",
            "bsec_compensated_humidity",
        ):
            self.garantir_coluna(self.config.tabela_ia, coluna, "DOUBLE NULL")

        for coluna in (
            "heater_profile",
            "duty_cycle",
            "bsec_config_nome",
            "bme_ai_studio_project",
            "mqtt_broker",
        ):
            self.garantir_coluna(self.config.tabela_ia, coluna, "VARCHAR(128) NULL")

        self.garantir_coluna(
            self.config.tabela_ia,
            "amostra_valida",
            "BOOLEAN NOT NULL DEFAULT TRUE",
        )
        self.garantir_coluna(
            self.config.tabela_ia,
            "motivo_invalidacao",
            "VARCHAR(255) NULL",
        )

        self.garantir_indice(self.config.tabela_ia, "idx_ia_sessao", "(`sessao_id`)")
        self.garantir_indice(self.config.tabela_ia, "idx_ia_classe", "(`classe`)")
        self.garantir_indice(self.config.tabela_ia, "idx_ia_fase", "(`fase`)")
        self.garantir_indice(self.config.tabela_ia, "idx_ia_recebido_em", "(`recebido_em`)")
        self.garantir_indice(self.config.tabela_ia, "idx_ia_broker", "(`mqtt_broker`)")

        # Tabela de inferências
        self.garantir_coluna(self.config.tabela_inferencias, "treinamento_id", "INT NULL")
        self.garantir_coluna(self.config.tabela_inferencias, "sessao_id", "VARCHAR(64) NULL")
        self.garantir_coluna(
            self.config.tabela_inferencias,
            "modelo_nome",
            "VARCHAR(128) NOT NULL DEFAULT 'modelo_indefinido'",
        )
        self.garantir_coluna(self.config.tabela_inferencias, "modelo_versao", "VARCHAR(64) NULL")
        self.garantir_coluna(
            self.config.tabela_inferencias,
            "origem_modelo",
            "VARCHAR(32) NOT NULL DEFAULT 'python'",
        )
        self.garantir_coluna(
            self.config.tabela_inferencias,
            "classe_predita",
            "VARCHAR(64) NOT NULL DEFAULT 'indefinida'",
        )
        self.garantir_coluna(self.config.tabela_inferencias, "classe_real", "VARCHAR(64) NULL")
        self.garantir_coluna(self.config.tabela_inferencias, "confianca", "DOUBLE NULL")

        for coluna in (
            "prob_ar_limpo",
            "prob_alcool",
            "prob_fumaca",
            "prob_cafe",
            "prob_ambiente_externo",
            "prob_outros",
            "temperatura",
            "pressao",
            "umidade",
            "gas",
            "altitude",
        ):
            self.garantir_coluna(self.config.tabela_inferencias, coluna, "DOUBLE NULL")

        self.garantir_coluna(self.config.tabela_inferencias, "payload_json", "TEXT NULL")
        self.garantir_coluna(self.config.tabela_inferencias, "observacao", "TEXT NULL")

        self.garantir_indice(self.config.tabela_inferencias, "idx_inf_modelo", "(`modelo_nome`)")
        self.garantir_indice(
            self.config.tabela_inferencias,
            "idx_inf_classe_predita",
            "(`classe_predita`)",
        )
        self.garantir_indice(
            self.config.tabela_inferencias,
            "idx_inf_inferido_em",
            "(`inferido_em`)",
        )
        self.garantir_indice(
            self.config.tabela_inferencias,
            "idx_inf_treinamento_id",
            "(`treinamento_id`)",
        )

    # -------------------------------------------------------------------------
    # INSERÇÕES
    # -------------------------------------------------------------------------

    def inserir_visualizacao(
        self,
        amostra: AmostraBME688,
        *,
        mqtt_broker: str | None,
        origem: str,
    ) -> int:
        dados = {
            coluna: valor
            for coluna, valor in amostra.visualizacao_dict().items()
            if valor is not None
        }

        if not dados:
            raise ValueError("amostra sem dados numéricos para visualização")

        dados["mqtt_broker"] = mqtt_broker
        dados["origem"] = origem

        colunas = ", ".join(f"`{coluna}`" for coluna in dados)
        placeholders = ", ".join(["%s"] * len(dados))
        valores = tuple(dados.values())

        with self._cursor() as cursor:
            cursor.execute(
                f"""
                INSERT INTO `{self.config.tabela_visualizacao}`
                ({colunas})
                VALUES ({placeholders})
                """,
                valores,
            )
            return int(cursor.lastrowid)

    def inserir_amostra_ia(
        self,
        amostra: AmostraBME688,
        *,
        mqtt_broker: str | None,
        origem: str,
    ) -> int | None:
        if not self.sessao.pronta_para_gravar():
            print("[IA] Amostra ignorada: sessão inativa ou classe não definida.")
            return None

        campos = {
            "sessao_id": self.sessao.sessao_id,
            "classe": self.sessao.classe,
            "fase": self.sessao.fase,
            "origem": origem if origem else self.sessao.origem,
            "operador": self.sessao.operador,
            "observacao": self.sessao.observacao,
            "temperatura": amostra.temperatura,
            "pressao": amostra.pressao,
            "umidade": amostra.umidade,
            "gas": amostra.gas,
            "altitude": amostra.altitude,
            "bsec_iaq": amostra.bsec_iaq,
            "bsec_static_iaq": amostra.bsec_static_iaq,
            "bsec_co2_equivalent": amostra.bsec_co2_equivalent,
            "bsec_breath_voc_equivalent": amostra.bsec_breath_voc_equivalent,
            "bsec_gas_percentage": amostra.bsec_gas_percentage,
            "bsec_compensated_temperature": amostra.bsec_compensated_temperature,
            "bsec_compensated_humidity": amostra.bsec_compensated_humidity,
            "heater_profile": amostra.heater_profile,
            "duty_cycle": amostra.duty_cycle,
            "bsec_config_nome": amostra.bsec_config_nome,
            "bme_ai_studio_project": amostra.bme_ai_studio_project,
            "mqtt_broker": mqtt_broker,
            "amostra_valida": True,
        }

        colunas = ", ".join(f"`{coluna}`" for coluna in campos)
        placeholders = ", ".join(["%s"] * len(campos))

        with self._cursor() as cursor:
            cursor.execute(
                f"""
                INSERT INTO `{self.config.tabela_ia}`
                ({colunas})
                VALUES ({placeholders})
                """,
                tuple(campos.values()),
            )
            self.sessao.amostras_coletadas += 1
            return int(cursor.lastrowid)

    def inserir_inferencia(
        self,
        *,
        modelo_nome: str,
        classe_predita: str,
        treinamento_id: int | None = None,
        sessao_id: str | None = None,
        modelo_versao: str | None = None,
        origem_modelo: str = "python",
        classe_real: str | None = None,
        confianca: float | None = None,
        probabilidades: dict[str, float | None] | None = None,
        amostra: AmostraBME688 | None = None,
        payload_json: dict[str, Any] | None = None,
        observacao: str | None = None,
    ) -> int:
        origem_modelo = origem_modelo.lower()

        if origem_modelo not in ORIGENS_MODELO_VALIDAS:
            raise ValueError(f"origem_modelo inválida: {origem_modelo}")

        probabilidades = probabilidades or {}
        amostra = amostra or AmostraBME688()

        campos = {
            "treinamento_id": treinamento_id,
            "sessao_id": sessao_id,
            "modelo_nome": modelo_nome,
            "modelo_versao": modelo_versao,
            "origem_modelo": origem_modelo,
            "classe_predita": classe_predita,
            "classe_real": classe_real,
            "confianca": confianca,
            "prob_ar_limpo": probabilidades.get("ar_limpo"),
            "prob_alcool": probabilidades.get("alcool"),
            "prob_fumaca": probabilidades.get("fumaca"),
            "prob_cafe": probabilidades.get("cafe"),
            "prob_ambiente_externo": probabilidades.get("ambiente_externo"),
            "prob_outros": probabilidades.get("outros"),
            "temperatura": amostra.temperatura,
            "pressao": amostra.pressao,
            "umidade": amostra.umidade,
            "gas": amostra.gas,
            "altitude": amostra.altitude,
            "payload_json": json.dumps(payload_json, ensure_ascii=False) if payload_json else None,
            "observacao": observacao,
        }

        colunas = ", ".join(f"`{coluna}`" for coluna in campos)
        placeholders = ", ".join(["%s"] * len(campos))

        with self._cursor() as cursor:
            cursor.execute(
                f"""
                INSERT INTO `{self.config.tabela_inferencias}`
                ({colunas})
                VALUES ({placeholders})
                """,
                tuple(campos.values()),
            )
            return int(cursor.lastrowid)

    def processar_amostra(
        self,
        amostra: AmostraBME688,
        *,
        origem: str,
        mqtt_broker: str | None,
    ) -> None:
        try:
            visualizacao_id = self.inserir_visualizacao(
                amostra,
                mqtt_broker=mqtt_broker,
                origem=origem,
            )

            ia_id = self.inserir_amostra_ia(
                amostra,
                mqtt_broker=mqtt_broker,
                origem=origem,
            )

            print(
                f"[BD] Visualização id={visualizacao_id}"
                + (f" | IA id={ia_id}" if ia_id else "")
                + f" | broker={mqtt_broker or '-'}"
                + f" | origem={origem}"
            )

        except Exception as exc:
            print(f"[ERRO] Falha ao gravar amostra ({origem}): {exc}")

    # -------------------------------------------------------------------------
    # CONSULTAS E EXPORTAÇÃO
    # -------------------------------------------------------------------------

    def exibir_visualizacao(self, limite: int = 10) -> None:
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
                mqtt_broker,
                origem
            FROM `{self.config.tabela_visualizacao}`
            ORDER BY id DESC
            LIMIT %s
            """
        )
        self._imprimir_tabela(
            titulo=f"{self.config.mysql_database}.{self.config.tabela_visualizacao}",
            query=query,
            params=(limite,),
            headers=(
                "ID",
                "TIMESTAMP",
                "TEMP",
                "PRESS",
                "UMID",
                "GAS",
                "ALT",
                "BROKER",
                "ORIGEM",
            ),
        )

    def exibir_ia(self, limite: int = 10) -> None:
        query = (
            f"""
            SELECT
                id,
                recebido_em,
                sessao_id,
                classe,
                fase,
                origem,
                temperatura,
                pressao,
                umidade,
                gas,
                altitude,
                mqtt_broker,
                amostra_valida
            FROM `{self.config.tabela_ia}`
            ORDER BY id DESC
            LIMIT %s
            """
        )
        self._imprimir_tabela(
            titulo=f"{self.config.mysql_database}.{self.config.tabela_ia}",
            query=query,
            params=(limite,),
            headers=(
                "ID",
                "TIMESTAMP",
                "SESSAO",
                "CLASSE",
                "FASE",
                "ORIGEM",
                "TEMP",
                "PRESS",
                "UMID",
                "GAS",
                "ALT",
                "BROKER",
                "VALIDA",
            ),
        )

    def exibir_inferencias(self, limite: int = 10) -> None:
        query = (
            f"""
            SELECT
                id,
                inferido_em,
                treinamento_id,
                sessao_id,
                modelo_nome,
                origem_modelo,
                classe_predita,
                classe_real,
                confianca
            FROM `{self.config.tabela_inferencias}`
            ORDER BY id DESC
            LIMIT %s
            """
        )
        self._imprimir_tabela(
            titulo=f"{self.config.mysql_database}.{self.config.tabela_inferencias}",
            query=query,
            params=(limite,),
            headers=(
                "ID",
                "TIMESTAMP",
                "TREINO_ID",
                "SESSAO",
                "MODELO",
                "ORIGEM",
                "PREDITA",
                "REAL",
                "CONF",
            ),
        )

    def exportar_ia_csv(self, caminho: str | Path | None = None) -> Path:
        try:
            import pandas as pd
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "Dependência ausente: pandas. Instale com: python -m pip install pandas"
            ) from exc

        destino = Path(caminho) if caminho else Path("exports") / (
            f"bme688_ia_treinamento_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        )

        if not destino.is_absolute():
            destino = BASE_DIR / destino

        destino.parent.mkdir(parents=True, exist_ok=True)

        query = (
            f"""
            SELECT
                recebido_em,
                sessao_id,
                classe,
                fase,
                origem,
                operador,
                temperatura,
                pressao,
                umidade,
                gas,
                altitude,
                bsec_iaq,
                bsec_static_iaq,
                bsec_co2_equivalent,
                bsec_breath_voc_equivalent,
                bsec_gas_percentage,
                bsec_compensated_temperature,
                bsec_compensated_humidity,
                heater_profile,
                duty_cycle,
                bsec_config_nome,
                bme_ai_studio_project,
                mqtt_broker
            FROM `{self.config.tabela_ia}`
            WHERE amostra_valida = TRUE
            ORDER BY recebido_em ASC
            """
        )

        with self.lock:
            self.conectar()
            assert self.conexao is not None
            df = pd.read_sql_query(query, self.conexao)

        df.to_csv(destino, index=False, encoding="utf-8")
        print(f"[EXPORT] Dataset IA exportado: {destino} ({len(df)} linhas)")

        return destino

    def apagar_visualizacao(self) -> None:
        with self._cursor() as cursor:
            cursor.execute(f"TRUNCATE TABLE `{self.config.tabela_visualizacao}`")
        print(f"[BD] Tabela `{self.config.tabela_visualizacao}` apagada.")

    def apagar_ia(self) -> None:
        with self._cursor() as cursor:
            cursor.execute(f"DELETE FROM `{self.config.tabela_ia}`")
        print(f"[BD] Registros da tabela `{self.config.tabela_ia}` apagados.")

    def apagar_inferencias(self) -> None:
        with self._cursor() as cursor:
            cursor.execute(f"DELETE FROM `{self.config.tabela_inferencias}`")
        print(f"[BD] Registros da tabela `{self.config.tabela_inferencias}` apagados.")

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

        print("\n" + "=" * 140)
        print(titulo)
        print("=" * 140)
        print(" | ".join(headers))
        print("-" * 140)

        for linha in linhas:
            print(" | ".join(formatar_celula(valor) for valor in linha))

        print("=" * 140 + "\n")

    def _cursor(self, dictionary: bool = False):
        return CursorTransacional(self, dictionary=dictionary)


class CursorTransacional:
    def __init__(self, banco: BancoBME688IA, dictionary: bool = False):
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
    prefixo = config.mqtt_topic_prefix

    return {
        f"{prefixo}/temperature": SensorTopic(
            topic=f"{prefixo}/temperature",
            coluna="temperatura",
            json_keys=("temperature", "temperature_c", "temperatura", "valor", "value"),
        ),
        f"{prefixo}/pressure": SensorTopic(
            topic=f"{prefixo}/pressure",
            coluna="pressao",
            json_keys=("pressure", "pressure_hpa", "pressao", "valor", "value"),
        ),
        f"{prefixo}/humidity": SensorTopic(
            topic=f"{prefixo}/humidity",
            coluna="umidade",
            json_keys=("humidity", "humidity_pct", "umidade", "valor", "value"),
        ),
        f"{prefixo}/gas": SensorTopic(
            topic=f"{prefixo}/gas",
            coluna="gas",
            json_keys=("gas", "gas_resistance", "gas_kohm", "gas_kohms", "valor", "value"),
        ),
        f"{prefixo}/altitude": SensorTopic(
            topic=f"{prefixo}/altitude",
            coluna="altitude",
            json_keys=("altitude", "altitude_m", "valor", "value"),
        ),
    }


def valor_float_topico(payload: str, topico: SensorTopic) -> float:
    dados_json = parse_json(payload)

    if dados_json:
        valor = extrair_primeiro_float(dados_json, topico.json_keys)

        if valor is not None:
            return valor

    valor = normalizar_float(payload)

    if valor is None:
        raise ValueError(f"payload sem valor numérico: {payload!r}")

    return valor


class ColetorMQTTBME688:
    def __init__(self, config: Config, banco: BancoBME688IA) -> None:
        self.config = config
        self.banco = banco

        self.topicos_sensores = criar_topicos_sensores(config)
        self.topico_comando = f"{config.mqtt_topic_prefix}/comando"
        self.topico_status = f"{config.mqtt_topic_prefix}/status"

        self.agregador = BufferAgregador(
            colunas_esperadas=len(self.topicos_sensores),
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

    def _criar_cliente_mqtt(self, broker: str) -> mqtt.Client:
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

        for item in self.topicos_sensores.values():
            client.subscribe(item.topic)

        client.subscribe(self.config.mqtt_topic_json)
        client.subscribe(self.config.mqtt_topic_aegis_json)
        client.subscribe(self.topico_status)
        client.subscribe(self.topico_comando)

        print(f"[MQTT][{broker}] Conectado.")
        print(f"[MQTT][{broker}] Prefixo ESP32: {self.config.mqtt_topic_prefix}")
        print(f"[MQTT][{broker}] JSON IA opcional: {self.config.mqtt_topic_json}")
        print(f"[MQTT][{broker}] JSON Aegis opcional: {self.config.mqtt_topic_aegis_json}")

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

        if topico == self.topico_status:
            print(f"[STATUS][{evento.broker}] {payload}")
            return

        if topico == self.topico_comando:
            print(
                f"[MQTT-CMD][{evento.broker}] "
                f"Comando recebido e ignorado por segurança: {payload}"
            )
            return

        if topico in (self.config.mqtt_topic_json, self.config.mqtt_topic_aegis_json):
            dados_json = parse_json(payload)
            if not dados_json:
                raise ValueError("payload JSON inválido para telemetria completa")

            amostra = AmostraBME688.from_json(dados_json)
            self.banco.processar_amostra(
                amostra,
                origem=f"mqtt_json:{topico}",
                mqtt_broker=evento.broker,
            )
            return

        topico_info = self.topicos_sensores.get(topico)
        if not topico_info:
            return

        valor = valor_float_topico(payload, topico_info)

        dados = self.agregador.adicionar_leitura(
            broker=evento.broker,
            coluna=topico_info.coluna,
            valor=valor,
        )

        print(f"[MQTT][{evento.broker}] {topico_info.coluna}={valor:.3f}")

        if dados:
            self.banco.processar_amostra(
                AmostraBME688.from_visualizacao_dict(dados),
                origem=f"mqtt_scalar:{self.config.mqtt_topic_prefix}",
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
  ver                         Exibe últimas leituras de dados_bme688_colunar
  ver ia                      Exibe últimas amostras de dados_bme688_ia_treinamento
  ver inferencias             Exibe últimas inferências

  sessao iniciar [id]         Inicia sessão de coleta IA
  sessao parar                Para gravação IA, mantendo visualização ativa
  sessao status               Mostra sessão atual

  classe <nome>               ar_limpo | alcool | fumaca | cafe | ambiente_externo
  fase <nome>                 baseline | exposicao | recuperacao | ambiente | indefinida
  origem <texto>              Define origem experimental
  operador <texto>            Define operador/responsável
  obs <texto>                 Define observação da sessão

  exportar ia [arquivo.csv]   Exporta dataset IA válido para CSV

  apagar visualizacao         Apaga somente dados_bme688_colunar, com confirmação
  apagar ia                   Apaga somente dados_bme688_ia_treinamento, com confirmação
  apagar inferencias          Apaga somente dados_bme688_ia_inferencias, com confirmação

  config                      Mostra a configuração ativa
  migrar                      Executa novamente as migrações de schema
  ajuda                       Mostra este painel
  sair                        Encerra o programa
"""
    )


def imprimir_config(config: Config) -> None:
    print("\n" + "=" * 80)
    print("CONFIGURAÇÃO ATIVA")
    print("=" * 80)
    print(f"MQTT brokers: {', '.join(config.mqtt_brokers)}")
    print(f"MQTT porta: {config.mqtt_port}")
    print(f"MQTT prefixo ESP32: {config.mqtt_topic_prefix}")
    print(f"MQTT JSON opcional: {config.mqtt_topic_json}")
    print(f"MQTT Aegis JSON opcional: {config.mqtt_topic_aegis_json}")
    print(f"MySQL host: {config.mysql_host}")
    print(f"MySQL database: {config.mysql_database}")
    print(f"Tabela visualização: {config.tabela_visualizacao}")
    print(f"Tabela IA: {config.tabela_ia}")
    print(f"Tabela inferências: {config.tabela_inferencias}")
    print("=" * 80 + "\n")


def executar_comando_cli(
    comando: str,
    banco: BancoBME688IA,
    sessao: SessaoColetaIA,
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

    if comando_lower in {"ver", "exibir"}:
        banco.exibir_visualizacao()
        return True

    if comando_lower == "ver ia":
        banco.exibir_ia()
        return True

    if comando_lower in {"ver inferencias", "ver inferência", "ver inferencia"}:
        banco.exibir_inferencias()
        return True

    if comando_lower.startswith("sessao iniciar"):
        partes = comando.split(maxsplit=2)
        sessao_id = partes[2].strip() if len(partes) >= 3 else None
        sessao.iniciar(sessao_id=sessao_id)
        print(f"[IA] Sessão iniciada: {sessao.status()}")
        return True

    if comando_lower == "sessao parar":
        sessao.parar()
        print(f"[IA] Sessão parada: {sessao.status()}")
        return True

    if comando_lower == "sessao status":
        print(f"[IA] {sessao.status()}")
        return True

    if comando_lower.startswith("classe "):
        sessao.definir_classe(comando_lower.split(maxsplit=1)[1])
        print(f"[IA] Classe atual: {sessao.classe}")
        return True

    if comando_lower.startswith("fase "):
        sessao.definir_fase(comando_lower.split(maxsplit=1)[1])
        print(f"[IA] Fase atual: {sessao.fase}")
        return True

    if comando_lower.startswith("origem "):
        sessao.origem = comando.split(maxsplit=1)[1].strip()
        print(f"[IA] Origem atual: {sessao.origem}")
        return True

    if comando_lower.startswith("operador "):
        sessao.operador = comando.split(maxsplit=1)[1].strip()
        print(f"[IA] Operador atual: {sessao.operador}")
        return True

    if comando_lower.startswith("obs "):
        sessao.observacao = comando.split(maxsplit=1)[1].strip()
        print(f"[IA] Observação atual: {sessao.observacao}")
        return True

    if comando_lower.startswith("exportar ia"):
        partes = comando.split(maxsplit=2)
        destino = partes[2].strip() if len(partes) >= 3 else None
        banco.exportar_ia_csv(destino)
        return True

    if comando_lower == "apagar visualizacao":
        frase = f"APAGAR {banco.config.tabela_visualizacao}"
        if confirmar_operacao(frase):
            banco.apagar_visualizacao()
        else:
            print("[BD] Operação cancelada.")
        return True

    if comando_lower == "apagar ia":
        frase = f"APAGAR {banco.config.tabela_ia}"
        if confirmar_operacao(frase):
            banco.apagar_ia()
        else:
            print("[BD] Operação cancelada.")
        return True

    if comando_lower == "apagar inferencias":
        frase = f"APAGAR {banco.config.tabela_inferencias}"
        if confirmar_operacao(frase):
            banco.apagar_inferencias()
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
    sessao = SessaoColetaIA()
    banco = BancoBME688IA(config, sessao)
    coletor = ColetorMQTTBME688(config, banco)

    print("\nAegisRover BME688 Data Pipeline IA")
    print(f"Banco: {config.mysql_database}")
    print(f"Tabela visualização: {config.tabela_visualizacao}")
    print(f"Tabela IA: {config.tabela_ia}")
    print(f"Tabela inferências: {config.tabela_inferencias}")

    imprimir_config(config)
    imprimir_ajuda()

    try:
        coletor.conectar()

        while True:
            comando = input("> ").strip()

            if not executar_comando_cli(comando, banco, sessao, config):
                break

    except KeyboardInterrupt:
        print("\n[APP] Finalizado via Ctrl+C.")

    finally:
        coletor.encerrar()
        banco.fechar()
        print("[APP] Sistema encerrado.")


if __name__ == "__main__":
    main()
