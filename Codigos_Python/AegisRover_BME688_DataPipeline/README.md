# AegisRover BME688 Data Pipeline

Pipeline local do AegisRover para coletar telemetria ambiental do BME688 via MQTT, gravar dados operacionais no MySQL e manter uma base separada para treinamento e inferência de IA.

## Objetivo

Separar corretamente três finalidades de dados:

| Tabela | Finalidade |
| --- | --- |
| `dados_bme688_colunar` | Visualização, dashboard, monitoramento e validação operacional. |
| `dados_bme688_ia_treinamento` | Dataset rotulado para treinamento, exportação CSV e rastreabilidade experimental. |
| `dados_bme688_ia_inferencias` | Registro de predições feitas por modelo Python, BSEC/BSEC2, embarcado ou manual. |

A tabela `dados_bme688_colunar` é preservada. Ela não foi renomeada, removida nem substituída.

## Estrutura

```text
AegisRover_BME688_DataPipeline/
├── bd_bme688_mqtt_ia.py
├── README.md
├── requirements.txt
├── .env.example
├── sql/
├── exports/
└── docs/
```

## Configuração

1. Instale as dependências:

```powershell
python -m pip install -r AegisRover_BME688_DataPipeline\requirements.txt
```

2. Copie `.env.example` para `.env`.

3. Ajuste as variáveis:

```env
MQTT_BROKER=test.mosquitto.org
MQTT_PORT=1883
MQTT_TOPIC_PREFIX=puc/iot/bme688
IOT_MYSQL_DATABASE=IoT
IOT_MYSQL_USER=root
IOT_MYSQL_PASSWORD=
```

Se `IOT_MYSQL_PASSWORD` não existir no ambiente, o script solicita a senha no terminal.

## Execução

```powershell
cd AegisRover_BME688_DataPipeline
python bd_bme688_mqtt_ia.py
```

## Tópicos MQTT

Compatibilidade preservada:

```text
puc/iot/bme688/temperatura
puc/iot/bme688/pressao
puc/iot/bme688/umidade
puc/iot/bme688/gas
puc/iot/bme688/altitude
```

Preferencial para treinamento de IA:

```text
puc/iot/bme688/telemetria
```

Também é aceito:

```text
aegis/sensor/bme688/sample
```

## Comandos da CLI

```text
ver
ver ia
ver inferencias
sessao iniciar [id]
sessao parar
sessao status
classe ar_limpo
classe alcool
classe fumaca
classe cafe
classe ambiente_externo
fase baseline
fase exposicao
fase recuperacao
origem aegis_rover_bancada
operador <nome>
obs <texto>
exportar ia [arquivo.csv]
apagar visualizacao
apagar ia
apagar inferencias
sair
```

Operações destrutivas exigem confirmação textual explícita e atuam em apenas uma tabela por vez.

## Sessão de coleta IA

Exemplo:

```text
classe ar_limpo
fase baseline
origem aegis_rover_bancada
obs coleta inicial em ambiente controlado
sessao iniciar S001
sessao status
sessao parar
```

Enquanto a sessão IA está ativa e a classe está definida, cada amostra válida é gravada em `dados_bme688_colunar` e também em `dados_bme688_ia_treinamento`. Fora da sessão, apenas a tabela de visualização recebe dados.

## Exportação CSV

Pela CLI:

```text
exportar ia
```

Ou com caminho específico:

```text
exportar ia exports\dataset_ar_limpo.csv
```

Pelo MySQL Workbench, use `sql/06_export_queries.sql` e exporte o Result Grid para CSV.

## BME AI-Studio e BSEC/BSEC2

O pipeline prepara campos para BSEC/BSEC2 e BME AI-Studio, mas não afirma que existe inferência treinada se não houver arquivo real exportado. Registre o nome do projeto e da configuração nos campos:

- `bme_ai_studio_project`
- `bsec_config_nome`

O arquivo exportado pelo BME AI-Studio/BSEC ainda precisa ser fornecido para que a integração com classificação treinada seja real.

## Compatibilidade SQL

Os scripts em `sql/` usam `ENUM` e `JSON` conforme MySQL moderno. O coletor Python tenta criar essas tabelas nesse formato e, se o servidor não aceitar esses tipos, usa fallback com `VARCHAR` e `TEXT`, preservando a semântica dos campos.
