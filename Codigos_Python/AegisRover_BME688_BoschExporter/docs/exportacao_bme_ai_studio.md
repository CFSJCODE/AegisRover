# Exportacao para BME AI-Studio

## Por que CSV comum nao basta

CSV comum preserva colunas tabulares, mas nao carrega a configuracao completa que o BME AI-Studio espera para interpretar uma coleta do BME688. O arquivo `.bmerawdata` agrega metadados de configuracao, perfil de aquecimento, ciclo de duty, cabecalho de dados brutos e bloco de amostras.

Nesta camada, o arquivo exportado contem quatro secoes:

- `configHeader`
- `configBody`
- `rawDataHeader`
- `rawDataBody`

## Diferenca entre tabelas

| Tabela | Uso correto |
| --- | --- |
| `dados_bme688_colunar` | Dashboard e monitoramento rapido. |
| `dados_bme688_ia_treinamento` | Dataset rotulado para modelos Python e analise academica. |
| `dados_bme688_bosch_raw` | Fonte fiel para exportacao `.bmerawdata`. |
| `dados_bme688_ia_inferencias` | Registro de predicoes e avaliacoes. |

## Exportacao

Execute a partir da pasta `AegisRover_BME688_BoschExporter`:

```powershell
python exporters\export_bmerawdata.py --sessao S001 --out exports\bmerawdata
```

O exportador:

1. le `dados_bme688_bosch_raw`;
2. filtra por `sessao_id`;
3. ordena por `timestamp_since_poweron`;
4. valida campos criticos;
5. valida sequencia temporal;
6. monta `configHeader`, `configBody`, `rawDataHeader` e `rawDataBody`;
7. grava `.bmerawdata`;
8. grava relatorio JSON.

## Validacao

```powershell
python exporters\validate_bmerawdata.py exports\bmerawdata\S001_ar_limpo.bmerawdata
```

O validador verifica:

- JSON valido;
- secoes obrigatorias;
- `dataColumns` exato;
- `dataBlock`;
- 13 campos por linha;
- tipos basicos;
- tempo crescente.

## Importacao no BME AI-Studio

Abra o BME AI-Studio e use o fluxo de importacao de dados brutos. Se o software rejeitar o arquivo, use o relatorio de validacao e compare o formato com uma exportacao nativa do BME AI-Studio/Development Kit.

## Limitacoes

O Aegis Rover usa um unico BME688. Isso permite coleta experimental real, mas nao reproduz integralmente o arranjo de multiplos sensores do Development Kit Bosch. A validacao final precisa considerar essa diferenca.
