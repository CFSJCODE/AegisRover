# MySQL Workbench - Bosch Raw

## Criar tabela

Use:

```sql
SOURCE sql/01_create_dados_bme688_bosch_raw.sql;
```

Ou abra o arquivo `sql/01_create_dados_bme688_bosch_raw.sql` no MySQL Workbench e execute.

## Consultar ultimos registros

```sql
USE IoT;

SELECT *
FROM dados_bme688_bosch_raw
ORDER BY id DESC
LIMIT 100;
```

## Resumo por sessao

```sql
SELECT
    sessao_id,
    classe,
    COUNT(*) AS total,
    MIN(timestamp_since_poweron) AS inicio_ms,
    MAX(timestamp_since_poweron) AS fim_ms,
    MIN(real_time_clock) AS inicio_unix,
    MAX(real_time_clock) AS fim_unix
FROM dados_bme688_bosch_raw
GROUP BY sessao_id, classe
ORDER BY inicio_unix DESC;
```

## Pontos por heater step

```sql
SELECT
    sessao_id,
    heater_profile_step_index,
    COUNT(*) AS total
FROM dados_bme688_bosch_raw
GROUP BY sessao_id, heater_profile_step_index
ORDER BY sessao_id, heater_profile_step_index;
```

## Pontos por ciclo

```sql
SELECT
    sessao_id,
    scanning_cycle_index,
    COUNT(*) AS pontos_no_ciclo
FROM dados_bme688_bosch_raw
GROUP BY sessao_id, scanning_cycle_index
ORDER BY sessao_id, scanning_cycle_index;
```

## Dados de uma sessao para exportacao

```sql
SELECT
    id,
    sessao_id,
    timestamp_since_poweron,
    real_time_clock,
    temperature,
    pressure,
    relative_humidity,
    resistance_gassensor,
    heater_profile_step_index,
    scanning_cycle_index,
    error_code
FROM dados_bme688_bosch_raw
WHERE sessao_id = 'S001'
ORDER BY timestamp_since_poweron ASC;
```

## Verificar campos criticos nulos

Use `sql/03_consultas_exportacao.sql` antes de exportar.

## Protecao das tabelas anteriores

Esta camada nao executa `DROP`, `TRUNCATE` ou alteracoes destrutivas nas tabelas:

- `dados_bme688_colunar`
- `dados_bme688_ia_treinamento`
- `dados_bme688_ia_inferencias`
