# Exportação para BME AI-Studio

## Por que CSV comum não basta

CSV comum preserva colunas tabulares, mas não carrega a configuração completa que o BME AI-Studio espera para interpretar uma coleta do BME688. O arquivo `.bmerawdata` agrega metadados de configuração, perfil de aquecimento, ciclo de duty, cabeçalho de dados brutos e bloco de amostras.

Nesta camada, o arquivo exportado contém quatro seções:

- `configHeader`
- `configBody`
- `rawDataHeader`
- `rawDataBody`

## Diferença entre tabelas

| Tabela | Uso correto |
| --- | --- |
| `dados_bme688_colunar` | Dashboard e monitoramento rápido. |
| `dados_bme688_ia_treinamento` | Dataset rotulado para modelos Python e análise acadêmica. |
| `dados_bme688_bosch_raw` | Fonte fiel para exportação `.bmerawdata`. |
| `dados_bme688_ia_inferencias` | Registro de predições e avaliações. |

## Exportação

Execute a partir da pasta `AegisRover_BME688_BoschExporter`:

```powershell
python exporters\export_bmerawdata.py --sessao S001 --out exports\bmerawdata
```

O exportador:

1. lê `dados_bme688_bosch_raw`;
2. filtra por `sessao_id`;
3. ordena por `timestamp_since_poweron`;
4. valida campos críticos;
5. valida a sequência temporal;
6. monta `configHeader`, `configBody`, `rawDataHeader` e `rawDataBody`;
7. grava `.bmerawdata`;
8. grava relatório JSON.

## Validação

```powershell
python exporters\validate_bmerawdata.py exports\bmerawdata\S001_ar_limpo.bmerawdata
```

O validador verifica:

- JSON válido;
- seções obrigatórias;
- `dataColumns` exato;
- `dataBlock`;
- 13 campos por linha;
- tipos básicos;
- tempo crescente.

## Importação no BME AI-Studio

Abra o BME AI-Studio e use o fluxo de importação de dados brutos. Se o software rejeitar o arquivo, use o relatório de validação e compare o formato com uma exportação nativa do BME AI-Studio/Development Kit.

## Limitações

O AegisRover usa um único BME688. Isso permite coleta experimental real, mas não reproduz integralmente o arranjo de múltiplos sensores do Development Kit Bosch. A validação final precisa considerar essa diferença.
