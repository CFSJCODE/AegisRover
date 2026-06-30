from __future__ import annotations

import argparse
import json
import re
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

BASE_DIR = Path(__file__).resolve().parents[1]

EXPECTED_SECTIONS = {"configHeader", "configBody", "rawDataHeader", "rawDataBody"}

EXPECTED_DATA_COLUMNS = [
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


def is_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def is_float(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def validar_linha(linha: Any, indice: int, erros: list[str]) -> None:
    if not isinstance(linha, list):
        erros.append(f"linha {indice}: deve ser array")
        return
    if len(linha) != 13:
        erros.append(f"linha {indice}: esperado 13 campos, recebido {len(linha)}")
        return

    checks = [
        (0, is_int, "Sensor Index"),
        (1, is_int, "Sensor ID"),
        (2, is_int, "Time Since PowerOn"),
        (3, is_int, "Real time clock"),
        (4, is_float, "Temperature"),
        (5, is_float, "Pressure"),
        (6, is_float, "Relative Humidity"),
        (7, is_float, "Resistance Gassensor"),
        (8, is_int, "Heater Profile Step Index"),
        (9, lambda v: isinstance(v, bool), "Scanning Mode Enabled"),
        (10, is_int, "Scanning Cycle Index"),
        (11, is_int, "Label Tag"),
        (12, is_int, "Error Code"),
    ]
    for pos, func, nome in checks:
        if not func(linha[pos]):
            erros.append(f"linha {indice}: campo {nome} possui tipo invalido: {linha[pos]!r}")

    if is_int(linha[8]) and not (0 <= linha[8] <= 9):
        erros.append(f"linha {indice}: Heater Profile Step Index fora de 0..9: {linha[8]}")


def validar_documento(documento: Any) -> tuple[list[str], list[str], int]:
    erros: list[str] = []
    avisos: list[str] = []

    if not isinstance(documento, dict):
        return ["arquivo JSON deve conter um objeto na raiz"], avisos, 0

    for secao in EXPECTED_SECTIONS:
        if secao not in documento:
            erros.append(f"secao ausente: {secao}")

    if erros:
        return erros, avisos, 0

    raw_body = documento.get("rawDataBody")
    if not isinstance(raw_body, dict):
        erros.append("rawDataBody deve ser objeto")
        return erros, avisos, 0

    data_columns = raw_body.get("dataColumns")
    data_block = raw_body.get("dataBlock")

    if data_columns != EXPECTED_DATA_COLUMNS:
        erros.append("rawDataBody.dataColumns nao corresponde exatamente ao schema esperado")

    if not isinstance(data_block, list):
        erros.append("rawDataBody.dataBlock deve ser array")
        return erros, avisos, 0

    for idx, linha in enumerate(data_block, start=1):
        validar_linha(linha, idx, erros)

    anterior = None
    for idx, linha in enumerate(data_block, start=1):
        if isinstance(linha, list) and len(linha) >= 3 and is_int(linha[2]):
            atual = linha[2]
            if anterior is not None and atual <= anterior:
                erros.append(
                    f"linha {idx}: Time Since PowerOn nao e crescente "
                    f"(anterior={anterior}, atual={atual})"
                )
            anterior = atual

    if not data_block:
        avisos.append("dataBlock vazio")

    return erros, avisos, len(data_block)


def sanitizar_nome(valor: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", valor).strip("_") or "arquivo"


def resolver_report_dir(path: str | Path) -> Path:
    destino = Path(path)
    if not destino.is_absolute():
        destino = BASE_DIR / destino
    destino.mkdir(parents=True, exist_ok=True)
    return destino


def validar_arquivo(path: str | Path, report_dir: str | Path | None) -> int:
    arquivo = Path(path)
    try:
        documento = json.loads(arquivo.read_text(encoding="utf-8"))
    except Exception as exc:
        print(f"[VALIDATE] JSON invalido: {exc}")
        return 1

    erros, avisos, total = validar_documento(documento)
    status = "error" if erros else "ok"

    print(f"[VALIDATE] Arquivo: {arquivo}")
    print(f"[VALIDATE] Status: {status}")
    print(f"[VALIDATE] Amostras: {total}")
    if erros:
        print("[VALIDATE] Erros:")
        for erro in erros:
            print(f"  - {erro}")
    if avisos:
        print("[VALIDATE] Avisos:")
        for aviso in avisos:
            print(f"  - {aviso}")

    if report_dir:
        relatorio = {
            "file": str(arquivo),
            "validated_at": datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z"),
            "status": status,
            "sample_count": total,
            "errors": erros,
            "warnings": avisos,
        }
        report_path = resolver_report_dir(report_dir) / f"{sanitizar_nome(arquivo.stem)}_validation_report.json"
        report_path.write_text(json.dumps(relatorio, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"[VALIDATE] Relatorio: {report_path}")

    return 1 if erros else 0


def main() -> None:
    parser = argparse.ArgumentParser(description="Valida arquivo .bmerawdata gerado para BME AI-Studio")
    parser.add_argument("arquivo", help="arquivo .bmerawdata")
    parser.add_argument("--report-dir", default="exports/reports", help="diretorio para relatorio JSON")
    parser.add_argument("--no-report", action="store_true", help="nao gerar relatorio JSON")
    args = parser.parse_args()
    raise SystemExit(validar_arquivo(args.arquivo, None if args.no_report else args.report_dir))


if __name__ == "__main__":
    main()
