USE IoT;

-- Verifica campos criticos nulos antes de exportar.
SELECT
    id,
    sessao_id,
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
    error_code
FROM dados_bme688_bosch_raw
WHERE sessao_id = 'S001'
  AND (
      sensor_index IS NULL
      OR sensor_id IS NULL
      OR timestamp_since_poweron IS NULL
      OR real_time_clock IS NULL
      OR temperature IS NULL
      OR pressure IS NULL
      OR relative_humidity IS NULL
      OR resistance_gassensor IS NULL
      OR heater_profile_step_index IS NULL
      OR scanning_enabled IS NULL
      OR scanning_cycle_index IS NULL
      OR label_tag IS NULL
      OR error_code IS NULL
  );

-- Detecta timestamps fora de ordem usando MySQL 8+.
SELECT *
FROM (
    SELECT
        id,
        sessao_id,
        timestamp_since_poweron,
        LAG(timestamp_since_poweron) OVER (
            PARTITION BY sessao_id
            ORDER BY timestamp_since_poweron, id
        ) AS timestamp_anterior
    FROM dados_bme688_bosch_raw
    WHERE sessao_id = 'S001'
) t
WHERE timestamp_anterior IS NOT NULL
  AND timestamp_since_poweron <= timestamp_anterior;

-- Dados que serao usados no dataBlock do .bmerawdata.
SELECT
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
    error_code
FROM dados_bme688_bosch_raw
WHERE sessao_id = 'S001'
ORDER BY timestamp_since_poweron ASC, id ASC;
