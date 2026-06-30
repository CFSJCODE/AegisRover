from __future__ import annotations

import argparse
import json
import os
import re
from dataclasses import dataclass
from datetime import datetime, timezone
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
        "Dependencia ausente: mysql-connector-python. "
        "Instale com: python -m pip install mysql-connector-python"
    ) from exc

BASE_DIR = Path(__file__).resolve().parents[1]
if load_dotenv:
    load_dotenv(BASE_DIR / ".env")

APP_VERSION = "custom-aegis-exporter-0.1.0"
DEFAULT_BOARD_MODE = "heater_profile_exploration"
DEFAULT_BOARD_LAYOUT = "grouped"

DATA_COLUMNS = [
    {"name": "Sensor Index", "unit": "", "format": "integer", "key": "sensor_index", "colId": 1},
    {"name": "Sensor ID", "unit": "", "format": "integer", "key": "sensor_id", "colId": 2},
    {"name": "Time Since PowerOn", "unit": "Milliseconds", "format": "integer", "key": "timestamp_since_poweron", "colId": 3},
    {"name": "Real time clock", "unit": "Unix Timestamp: seconds since Jan 01 1970. (UTC); 0 = missing", "format": "integer", "key": "real_time_clock", "colId": 4},
    {"name": "Temperature", "unit": "DegreesCelcius", "format": "float", "key": "temperature", "colId": 5},
    {"name": "Pressure", "unit": "Hectopascals", "format": "float", "key": "pressure", "colId": 6},
    {"name": "Relative Humidity", "unit": "Percent", "format": "float", "key": "relative_humidity", "colId": 7},
    {"name": "Resistance Gassensor", "unit": "Ohms", "format": "float", "key": "resistance_gassensor", "colId": 8},
    {"name": "Heater Profile Step Index", "unit": "", "format": "integer", "key": "heater_profile_step_index", "colId": 9},
    {"name": "Scanning Mode Enabled", "unit": "", "format": "boolean", "key": "scanning_enabled", "colId": 10},
    {"name": "Scanning Cycle Index", "unit": "", "format": "integer", "key": "scanning_cycle_index", "colId": 11},
    {"name": "Label Tag", "unit": "", "format": "integer", "key": "label_tag", "colId": 12},
    {"name": "Error Code", "unit": "", "format": "integer", "key": "error_code", "colId": 13},
]

CRITICAL_KEYS = [column["key"] for column in DATA_COLUMNS]


def validar_identificador_mysql(valor: str, nome: str) -> str:
    if not re.fullmatch(r"[A-Za-z0-9_]+", valor):
        raise ValueError(f"{nome} deve conter apenas letras, numeros e underscore: {valor!r}")
    return valor


@dataclass(frozen=True)
class Config:
    mysql_host: str
    mysql_database: str
    mysql_user: str
    mysql_password: str | None
    tabela_raw: str

    @classmethod
    def from_env(cls) -> Config:
        return cls(
            mysql_host=os.getenv("IOT_MYSQL_HOST", "localhost"),
            mysql_database=validar_identificador_mysql(
                os.getenv("IOT_MYSQL_DATABASE", "IoT"), "IOT_MYSQL_DATABASE"
            ),
            mysql_user=os.getenv("IOT_MYSQL_USER", "root"),
            mysql_password=os.getenv("IOT_MYSQL_PASSWORD"),
            tabela_raw=validar_identificador_mysql(
                os.getenv("IOT_MYSQL_TABLE_BOSCH_RAW", "dados_bme688_bosch_raw"),
                "IOT_MYSQL_TABLE_BOSCH_RAW",
            ),
        )


def conectar_mysql(config: Config):
    senha = config.mysql_password
    if senha is None:
        senha = getpass(f"Senha MySQL para {config.mysql_user}@{config.mysql_host}: ")
    conexao = mysql.connector.connect(
        host=config.mysql_host,
        user=config.mysql_user,
        password=senha,
        database=config.mysql_database,
    )
    return conexao


def carregar_registros(config: Config, sessao_id: str) -> list[dict[str, Any]]:
    query = f"""
        SELECT
            id,
            sessao_id,
            classe,
            fase,
            sensor_index,
            sensor_id,
            timestamp_since_poweron,
            real_time_clock,
            temperature,
            pressure,
            relative_humidity,
            resistance_gassensor,
            heater_profile_step_index,
            scanning_enabled,
            scanning_cycle_index,
            label_tag,
            error_code,
            heater_profile_id,
            duty_cycle_profile_id,
            board_type,
            board_id,
            firmware_version,
            recebido_em
        FROM `{config.tabela_raw}`
        WHERE sessao_id = %s
        ORDER BY timestamp_since_poweron ASC, id ASC
    """
    conexao = conectar_mysql(config)
    try:
        cursor = conexao.cursor(dictionary=True)
        cursor.execute(query, (sessao_id,))
        registros = list(cursor.fetchall())
        cursor.close()
        return registros
    finally:
        conexao.close()


def validar_registros(registros: list[dict[str, Any]]) -> tuple[list[str], list[str]]:
    erros: list[str] = []
    avisos: list[str] = []

    if not registros:
        erros.append("sessao sem amostras")
        return erros, avisos

    for linha in registros:
        for chave in CRITICAL_KEYS:
            if linha.get(chave) is None:
                erros.append(f"id={linha.get('id')} campo critico nulo: {chave}")

        step = linha.get("heater_profile_step_index")
        if step is not None and not (0 <= int(step) <= 9):
            erros.append(f"id={linha.get('id')} heater_profile_step_index fora de 0..9: {step}")

        if linha.get("scanning_cycle_index") is None:
            erros.append(f"id={linha.get('id')} scanning_cycle_index ausente")

    anterior = None
    for linha in registros:
        atual = int(linha["timestamp_since_poweron"])
        if anterior is not None and atual <= anterior:
            erros.append(
                "timestamp_since_poweron deve ser estritamente crescente: "
                f"anterior={anterior}, atual={atual}, id={linha.get('id')}"
            )
        anterior = atual

    for chave in ("board_type", "board_id", "heater_profile_id", "duty_cycle_profile_id", "sensor_index"):
        valores = {linha.get(chave) for linha in registros}
        if len(valores) > 1:
            erros.append(f"sessao possui multiplos valores para {chave}: {sorted(str(v) for v in valores)}")

    sensor_index = registros[0].get("sensor_index")
    if sensor_index != 0:
        erros.append(f"configuracao inicial suporta sensor_index 0; encontrado {sensor_index}")

    heater_profile = registros[0].get("heater_profile_id")
    duty_cycle = registros[0].get("duty_cycle_profile_id")
    if heater_profile != "heater_354":
        avisos.append(f"heater_profile_id diferente do perfil inicial documentado: {heater_profile}")
    if duty_cycle != "duty_1":
        avisos.append(f"duty_cycle_profile_id diferente do duty inicial documentado: {duty_cycle}")

    return erros, avisos


def iso_utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def unix_now_ms() -> int:
    return int(datetime.now(timezone.utc).timestamp() * 1000)


def montar_bmerawdata(registros: list[dict[str, Any]]) -> dict[str, Any]:
    primeiro = registros[0]
    date_created_iso = iso_utc_now()
    date_created_ms = unix_now_ms()

    data_block = []
    for linha in registros:
        data_block.append(
            [
                int(linha["sensor_index"]),
                int(linha["sensor_id"]),
                int(linha["timestamp_since_poweron"]),
                int(linha["real_time_clock"]),
                float(linha["temperature"]),
                float(linha["pressure"]),
                float(linha["relative_humidity"]),
                float(linha["resistance_gassensor"]),
                int(linha["heater_profile_step_index"]),
                bool(linha["scanning_enabled"]),
                int(linha["scanning_cycle_index"]),
                int(linha["label_tag"]),
                int(linha["error_code"]),
            ]
        )

    return {
        "configHeader": {
            "dateCreated_ISO": date_created_iso,
            "appVersion": APP_VERSION,
            "boardType": primeiro["board_type"],
            "boardMode": DEFAULT_BOARD_MODE,
            "boardLayout": DEFAULT_BOARD_LAYOUT,
        },
        "configBody": {
            "heaterProfiles": [
                {
                    "id": "heater_354",
                    "timeBase": 140,
                    "temperatureTimeVectors": [
                        [320, 5],
                        [100, 2],
                        [100, 10],
                        [100, 30],
                        [200, 5],
                        [200, 5],
                        [200, 5],
                        [320, 5],
                        [320, 5],
                        [320, 5],
                    ],
                }
            ],
            "dutyCycleProfiles": [
                {
                    "id": "duty_1",
                    "numberScanningCycles": 1,
                    "numberSleepingCycles": 0,
                }
            ],
            "sensorConfigurations": [
                {
                    "sensorIndex": 0,
                    "heaterProfile": "heater_354",
                    "dutyCycleProfile": "duty_1",
                }
            ],
        },
        "rawDataHeader": {
            "counterPowerOnOff": 1,
            "seedPowerOnOff": 1,
            "counterFileLimit": 0,
            "dateCreated": date_created_ms,
            "dateCreated_ISO": date_created_iso,
            "firmwareVersion": primeiro.get("firmware_version"),
            "boardId": primeiro["board_id"],
        },
        "rawDataBody": {
            "dataColumns": DATA_COLUMNS,
            "dataBlock": data_block,
        },
    }


def sanitizar_nome(valor: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", valor).strip("_") or "sessao"


def resolver_saida(path: str | Path) -> Path:
    destino = Path(path)
    if not destino.is_absolute():
        destino = BASE_DIR / destino
    destino.mkdir(parents=True, exist_ok=True)
    return destino


def salvar_relatorio(report_dir: Path, sessao_id: str, relatorio: dict[str, Any]) -> Path:
    report_dir.mkdir(parents=True, exist_ok=True)
    path = report_dir / f"{sanitizar_nome(sessao_id)}_export_report.json"
    path.write_text(json.dumps(relatorio, indent=2, ensure_ascii=False), encoding="utf-8")
    return path


def exportar(sessao_id: str, out_dir: str | Path, report_dir: str | Path, pretty: bool) -> int:
    config = Config.from_env()
    registros = carregar_registros(config, sessao_id)
    erros, avisos = validar_registros(registros)
    relatorio: dict[str, Any] = {
        "sessao_id": sessao_id,
        "table": config.tabela_raw,
        "sample_count": len(registros),
        "errors": erros,
        "warnings": avisos,
        "status": "error" if erros else "ok",
    }

    report_path = salvar_relatorio(resolver_saida(report_dir), sessao_id, relatorio)
    if erros:
        print("[EXPORT] Falha na validacao:")
        for erro in erros:
            print(f"  - {erro}")
        print(f"[EXPORT] Relatorio: {report_path}")
        return 1

    documento = montar_bmerawdata(registros)
    destino_dir = resolver_saida(out_dir)
    classe = registros[0].get("classe") or "sem_classe"
    arquivo = destino_dir / f"{sanitizar_nome(sessao_id)}_{sanitizar_nome(str(classe))}.bmerawdata"
    arquivo.write_text(
        json.dumps(documento, indent=2 if pretty else None, ensure_ascii=False),
        encoding="utf-8",
    )

    relatorio.update(
        {
            "status": "ok",
            "output_file": str(arquivo),
            "report_file": str(report_path),
            "first_timestamp_since_poweron": registros[0]["timestamp_since_poweron"],
            "last_timestamp_since_poweron": registros[-1]["timestamp_since_poweron"],
            "board_id": registros[0]["board_id"],
            "firmware_version": registros[0].get("firmware_version"),
        }
    )
    report_path.write_text(json.dumps(relatorio, indent=2, ensure_ascii=False), encoding="utf-8")

    print(f"[EXPORT] Arquivo gerado: {arquivo}")
    print(f"[EXPORT] Amostras: {len(registros)}")
    if avisos:
        print("[EXPORT] Avisos:")
        for aviso in avisos:
            print(f"  - {aviso}")
    print(f"[EXPORT] Relatorio: {report_path}")
    return 0


def main() -> None:
    parser = argparse.ArgumentParser(description="Exporta dados BME688 Bosch raw para .bmerawdata")
    parser.add_argument("--sessao", required=True, help="sessao_id a exportar")
    parser.add_argument("--out", default="exports/bmerawdata", help="diretorio de saida")
    parser.add_argument("--report-dir", default="exports/reports", help="diretorio de relatorios")
    parser.add_argument("--compact", action="store_true", help="gera JSON compacto")
    args = parser.parse_args()
    raise SystemExit(exportar(args.sessao, args.out, args.report_dir, pretty=not args.compact))


if __name__ == "__main__":
    main()
