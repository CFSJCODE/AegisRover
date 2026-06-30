# AegisRover BME688 Bosch Exporter

Camada adicional do AegisRover para coletar dados brutos estruturados do BME688 e exportar arquivos `.bmerawdata` para uso no BME AI-Studio.

Esta pasta nao substitui o pipeline anterior `AegisRover_BME688_DataPipeline`. Ela trabalha em paralelo e usa uma tabela exclusiva:

```text
dados_bme688_bosch_raw
```

## Estrutura

```text
AegisRover_BME688_BoschExporter/
├── collectors/
│   └── bd_bme688_mqtt_raw.py
├── exporters/
│   ├── export_bmerawdata.py
│   └── validate_bmerawdata.py
├── sql/
├── exports/
│   ├── bmerawdata/
│   └── reports/
└── docs/
```

## Finalidade das tabelas do projeto

| Tabela | Finalidade |
| --- | --- |
| `dados_bme688_colunar` | Visualizacao e monitoramento operacional. |
| `dados_bme688_ia_treinamento` | Dataset rotulado para IA em Python e rastreabilidade academica. |
| `dados_bme688_bosch_raw` | Dados brutos estruturados para exportacao `.bmerawdata`. |
| `dados_bme688_ia_inferencias` | Historico de inferencias de modelos. |

## Instalacao

```powershell
cd "D:\Acadêmico\Faculdade - PUC\3º Semestre - 01 2026\Internet Das Coisas I - IOT I\Trabalho Prático\AegisRover\AegisRover_BME688_BoschExporter"
python -m pip install -r requirements.txt
```

Copie `.env.example` para `.env` e ajuste credenciais. Se `IOT_MYSQL_PASSWORD` estiver vazio ou ausente, os scripts solicitam a senha no terminal.

## Coletor MQTT raw

Topico esperado:

```text
puc/iot/bme688/raw
```

Executar:

```powershell
python collectors\bd_bme688_mqtt_raw.py
```

O coletor cria `IoT.dados_bme688_bosch_raw` se a tabela ainda nao existir e grava somente payloads JSON completos e validos.

## Payload MQTT esperado

```json
{
  "sessao_id": "S001",
  "classe": "ar_limpo",
  "fase": "baseline",
  "sensor_index": 0,
  "sensor_id": 10068801,
  "timestamp_since_poweron": 215430,
  "real_time_clock": 1781970000,
  "temperature": 27.81,
  "pressure": 914.62,
  "relative_humidity": 58.24,
  "resistance_gassensor": 3719409.75,
  "heater_profile_step_index": 0,
  "scanning_enabled": true,
  "scanning_cycle_index": 1,
  "label_tag": 0,
  "error_code": 0,
  "heater_profile_id": "heater_354",
  "duty_cycle_profile_id": "duty_1",
  "board_type": "aegis_rover_single_bme688",
  "board_id": "AegisRover_BME688_01",
  "firmware_version": "aegis-bme688-raw-0.1.0"
}
```

## Exportar `.bmerawdata`

```powershell
python exporters\export_bmerawdata.py --sessao S001 --out exports\bmerawdata
```

O arquivo gerado contem:

```json
{
  "configHeader": {},
  "configBody": {},
  "rawDataHeader": {},
  "rawDataBody": {}
}
```

O exportador tambem gera relatorio JSON em `exports/reports/`.

## Validar `.bmerawdata`

```powershell
python exporters\validate_bmerawdata.py exports\bmerawdata\S001_ar_limpo.bmerawdata
```

O validador verifica secoes obrigatorias, `dataColumns`, `dataBlock`, 13 campos por linha, tipos basicos e sequencia temporal.

## Limitacoes tecnicas

- Esta camada gera uma estrutura `.bmerawdata` customizada e minima, alinhada aos campos solicitados para o projeto. A aceitacao final no BME AI-Studio deve ser validada no proprio software.
- Um unico BME688 no Aegis Rover nao equivale ao Development Kit Bosch com multiplos sensores.
- O firmware precisa publicar `sensor_id`, `timestamp_since_poweron`, `real_time_clock`, `heater_profile_step_index` e `scanning_cycle_index`; o coletor nao inventa esses valores.
- O perfil inicial documentado e `heater_354` com 10 passos e `duty_1`.

## Garantias desta fase

- Nao altera `dados_bme688_colunar`.
- Nao altera `dados_bme688_ia_treinamento`.
- Nao altera `dados_bme688_ia_inferencias`.
- Nao altera `Banco De Dados\bd_bme688_mqtt.py`.
- Nao altera `AegisRover_BME688_DataPipeline\bd_bme688_mqtt_ia.py`.
- Nao executa upload para GitHub.
