# Pipeline de IA para BME688

## Fluxo geral

```text
ESP32 / Aegis Rover / BME688
↓
MQTT
↓
Script Python coletor
↓
MySQL
├── dados_bme688_colunar
├── dados_bme688_ia_treinamento
└── dados_bme688_ia_inferencias
↓
CSV / pandas DataFrame
↓
BME AI-Studio / BSEC / BSEC2 / modelos Python
↓
Inferências registradas no banco
```

## Diferença entre camadas

| Camada | Significado |
| --- | --- |
| Leitura bruta BME688 | Temperatura, umidade, pressão, gás e altitude estimada. |
| BSEC/BSEC2 | Processamento Bosch com saídas como IAQ, CO2 equivalente e VOC equivalente. |
| Dataset rotulado | Amostras com sessão, classe, fase, origem e observação. |
| Inferência | Resultado de um modelo aplicado a uma amostra. |

## Tabela de visualização

`dados_bme688_colunar` deve ser usada para dashboard, validação operacional e acompanhamento rápido. Ela não deve ser a tabela principal de treinamento.

## Tabela de treinamento

`dados_bme688_ia_treinamento` reúne amostras rotuladas. Os campos `sessao_id`, `classe` e `fase` são essenciais para rastreabilidade acadêmica.

Campos BSEC podem ficar nulos enquanto o firmware publicar apenas leitura bruta.

## Tabela de inferências

`dados_bme688_ia_inferencias` registra resultados de modelos. Use `treinamento_id` quando a inferência estiver associada a uma amostra da tabela de treinamento.

## Uso com pandas, scikit-learn e PyTorch

Exporte o dataset:

```text
exportar ia
```

Depois carregue em Python:

```python
import pandas as pd

df = pd.read_csv("exports/bme688_ia_treinamento.csv")
df = df[df["classe"].notna()]
```

Separe sessões ao criar os conjuntos de treinamento e teste para evitar vazamento experimental. Uma estratégia conservadora é reservar sessões inteiras para teste.

## Uso com BME AI-Studio/BSEC/BSEC2

O BME AI-Studio pode apoiar a criação de configuração/modelo para classificação com BME688. O pipeline registra os metadados necessários, mas a inferência real depende de arquivo exportado pelo BME AI-Studio/BSEC.

Campos de rastreabilidade:

- `bsec_config_nome`
- `bme_ai_studio_project`
- `heater_profile`
- `duty_cycle`
