# MySQL Workbench - Comandos do Pipeline BME688

## Tabelas

| Tabela | Finalidade |
| --- | --- |
| `dados_bme688_colunar` | Visualização e monitoramento ambiental. |
| `dados_bme688_ia_treinamento` | Dataset rotulado para treinamento de IA. |
| `dados_bme688_ia_inferencias` | Histórico de inferências e avaliações de modelos. |

## Criar banco e tabelas

Use `sql/01_create_tables.sql`.

O script Python possui fallback automático para servidores que não aceitem `ENUM` ou `JSON`, usando `VARCHAR` e `TEXT`. No Workbench, prefira o script completo se estiver usando MySQL 8.x.

```sql
CREATE DATABASE IF NOT EXISTS IoT
CHARACTER SET utf8mb4
COLLATE utf8mb4_unicode_ci;

USE IoT;
SHOW TABLES;
```

## Descrever tabelas

```sql
DESCRIBE dados_bme688_colunar;
DESCRIBE dados_bme688_ia_treinamento;
DESCRIBE dados_bme688_ia_inferencias;
```

## Visualizar últimos dados de visualização

```sql
SELECT
    id,
    recebido_em,
    temperatura,
    pressao,
    umidade,
    gas,
    altitude
FROM dados_bme688_colunar
ORDER BY id DESC
LIMIT 100;
```

## Visualizar últimos dados de IA

```sql
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
    heater_profile,
    duty_cycle,
    amostra_valida
FROM dados_bme688_ia_treinamento
ORDER BY id DESC
LIMIT 100;
```

## Visualizar inferências

```sql
SELECT
    id,
    inferido_em,
    treinamento_id,
    sessao_id,
    modelo_nome,
    modelo_versao,
    origem_modelo,
    classe_predita,
    classe_real,
    confianca,
    temperatura,
    pressao,
    umidade,
    gas,
    altitude
FROM dados_bme688_ia_inferencias
ORDER BY id DESC
LIMIT 100;
```

## Contagens

```sql
SELECT COUNT(*) AS total_visualizacao
FROM dados_bme688_colunar;

SELECT COUNT(*) AS total_ia_treinamento
FROM dados_bme688_ia_treinamento;

SELECT COUNT(*) AS total_inferencias
FROM dados_bme688_ia_inferencias;
```

## Amostras por classe

```sql
SELECT
    classe,
    COUNT(*) AS total
FROM dados_bme688_ia_treinamento
GROUP BY classe
ORDER BY total DESC;
```

## Amostras por sessão

```sql
SELECT
    sessao_id,
    classe,
    fase,
    COUNT(*) AS total_amostras,
    MIN(recebido_em) AS inicio,
    MAX(recebido_em) AS fim
FROM dados_bme688_ia_treinamento
GROUP BY sessao_id, classe, fase
ORDER BY inicio DESC;
```

## Dados válidos para treinamento

```sql
SELECT
    id,
    recebido_em,
    sessao_id,
    classe,
    fase,
    temperatura,
    pressao,
    umidade,
    gas,
    altitude,
    bsec_iaq,
    bsec_static_iaq,
    bsec_co2_equivalent,
    bsec_breath_voc_equivalent
FROM dados_bme688_ia_treinamento
WHERE amostra_valida = TRUE
ORDER BY recebido_em ASC;
```

## Dados inválidos

```sql
SELECT
    id,
    recebido_em,
    sessao_id,
    classe,
    fase,
    motivo_invalidacao
FROM dados_bme688_ia_treinamento
WHERE amostra_valida = FALSE
ORDER BY recebido_em DESC;
```

## Invalidar amostra específica

```sql
UPDATE dados_bme688_ia_treinamento
SET
    amostra_valida = FALSE,
    motivo_invalidacao = 'Leitura instável durante transição de exposição'
WHERE id = 1;
```

## Corrigir classe de sessão

```sql
UPDATE dados_bme688_ia_treinamento
SET classe = 'alcool'
WHERE sessao_id = 'S001';
```

## Corrigir fase em intervalo

```sql
UPDATE dados_bme688_ia_treinamento
SET fase = 'baseline'
WHERE sessao_id = 'S001'
  AND recebido_em BETWEEN '2026-06-20 14:00:00' AND '2026-06-20 14:03:00';
```

## Médias por classe

```sql
SELECT
    classe,
    COUNT(*) AS n,
    AVG(temperatura) AS media_temperatura,
    AVG(umidade) AS media_umidade,
    AVG(pressao) AS media_pressao,
    AVG(gas) AS media_gas,
    AVG(altitude) AS media_altitude
FROM dados_bme688_ia_treinamento
WHERE amostra_valida = TRUE
GROUP BY classe
ORDER BY classe;
```

## Inferências por modelo

```sql
SELECT
    modelo_nome,
    modelo_versao,
    origem_modelo,
    COUNT(*) AS total_inferencias,
    AVG(confianca) AS confianca_media
FROM dados_bme688_ia_inferencias
GROUP BY modelo_nome, modelo_versao, origem_modelo
ORDER BY total_inferencias DESC;
```

## Acurácia quando houver classe real

```sql
SELECT
    modelo_nome,
    COUNT(*) AS total_avaliado,
    SUM(CASE WHEN classe_predita = classe_real THEN 1 ELSE 0 END) AS acertos,
    ROUND(
        100.0 * SUM(CASE WHEN classe_predita = classe_real THEN 1 ELSE 0 END) / COUNT(*),
        2
    ) AS acuracia_percentual
FROM dados_bme688_ia_inferencias
WHERE classe_real IS NOT NULL
GROUP BY modelo_nome
ORDER BY acuracia_percentual DESC;
```

## Comparar inferências com dataset

```sql
SELECT
    i.id AS inferencia_id,
    i.inferido_em,
    i.modelo_nome,
    i.classe_predita,
    i.classe_real,
    i.confianca,
    t.id AS amostra_id,
    t.sessao_id,
    t.classe AS classe_dataset,
    t.fase,
    t.temperatura,
    t.umidade,
    t.pressao,
    t.gas
FROM dados_bme688_ia_inferencias i
LEFT JOIN dados_bme688_ia_treinamento t
    ON i.treinamento_id = t.id
ORDER BY i.inferido_em DESC
LIMIT 100;
```

## Exportar CSV no Workbench

Execute:

```sql
SELECT
    recebido_em,
    sessao_id,
    classe,
    fase,
    temperatura,
    pressao,
    umidade,
    gas,
    altitude,
    bsec_iaq,
    bsec_static_iaq,
    bsec_co2_equivalent,
    bsec_breath_voc_equivalent,
    heater_profile,
    duty_cycle
FROM dados_bme688_ia_treinamento
WHERE amostra_valida = TRUE
ORDER BY recebido_em ASC;
```

Depois use a opção de exportação do Result Grid para CSV.

## Comandos destrutivos protegidos

Não execute sem intenção clara:

```sql
TRUNCATE TABLE dados_bme688_colunar;
DELETE FROM dados_bme688_ia_inferencias;
DELETE FROM dados_bme688_ia_treinamento;
```

Para remover apenas uma sessão:

```sql
DELETE FROM dados_bme688_ia_treinamento
WHERE sessao_id = 'S001';
```
