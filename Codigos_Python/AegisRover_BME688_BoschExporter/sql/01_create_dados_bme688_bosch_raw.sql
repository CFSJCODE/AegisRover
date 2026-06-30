CREATE DATABASE IF NOT EXISTS IoT
CHARACTER SET utf8mb4
COLLATE utf8mb4_unicode_ci;

USE IoT;

CREATE TABLE IF NOT EXISTS dados_bme688_bosch_raw (
    id BIGINT AUTO_INCREMENT PRIMARY KEY,

    sessao_id VARCHAR(64) NOT NULL,
    classe VARCHAR(64) NULL,
    fase VARCHAR(32) NULL,

    sensor_index INT NOT NULL DEFAULT 0,
    sensor_id BIGINT NOT NULL,

    timestamp_since_poweron BIGINT NOT NULL,
    real_time_clock BIGINT NOT NULL,

    temperature DOUBLE NOT NULL,
    pressure DOUBLE NOT NULL,
    relative_humidity DOUBLE NOT NULL,
    resistance_gassensor DOUBLE NOT NULL,

    heater_profile_step_index INT NOT NULL,
    scanning_enabled BOOLEAN NOT NULL DEFAULT TRUE,
    scanning_cycle_index INT NOT NULL,
    label_tag INT NOT NULL DEFAULT 0,
    error_code INT NOT NULL DEFAULT 0,

    heater_profile_id VARCHAR(64) NOT NULL DEFAULT 'heater_354',
    duty_cycle_profile_id VARCHAR(64) NOT NULL DEFAULT 'duty_1',

    board_type VARCHAR(64) NOT NULL DEFAULT 'aegis_rover_single_bme688',
    board_id VARCHAR(128) NOT NULL DEFAULT 'AegisRover_BME688_01',
    firmware_version VARCHAR(64) NULL,

    recebido_em TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,

    INDEX idx_raw_sessao (sessao_id),
    INDEX idx_raw_classe (classe),
    INDEX idx_raw_poweron (timestamp_since_poweron),
    INDEX idx_raw_rtc (real_time_clock),
    INDEX idx_raw_cycle (scanning_cycle_index, heater_profile_step_index)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

SHOW TABLES;
DESCRIBE dados_bme688_bosch_raw;
