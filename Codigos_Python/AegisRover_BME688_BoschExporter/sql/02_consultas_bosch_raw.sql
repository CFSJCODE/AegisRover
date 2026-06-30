USE IoT;

SELECT *
FROM dados_bme688_bosch_raw
ORDER BY id DESC
LIMIT 100;

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

SELECT
    sessao_id,
    heater_profile_step_index,
    COUNT(*) AS total
FROM dados_bme688_bosch_raw
GROUP BY sessao_id, heater_profile_step_index
ORDER BY sessao_id, heater_profile_step_index;

SELECT
    sessao_id,
    scanning_cycle_index,
    COUNT(*) AS pontos_no_ciclo
FROM dados_bme688_bosch_raw
GROUP BY sessao_id, scanning_cycle_index
ORDER BY sessao_id, scanning_cycle_index;

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
