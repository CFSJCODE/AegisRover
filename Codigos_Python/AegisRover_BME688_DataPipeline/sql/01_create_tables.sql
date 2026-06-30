CREATE DATABASE IF NOT EXISTS IoT
CHARACTER SET utf8mb4
COLLATE utf8mb4_unicode_ci;

USE IoT;

CREATE TABLE IF NOT EXISTS dados_bme688_colunar (
    id INT AUTO_INCREMENT PRIMARY KEY,
    recebido_em TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    temperatura DOUBLE NULL,
    pressao DOUBLE NULL,
    umidade DOUBLE NULL,
    gas DOUBLE NULL,
    altitude DOUBLE NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

CREATE TABLE IF NOT EXISTS dados_bme688_ia_treinamento (
    id INT AUTO_INCREMENT PRIMARY KEY,
    recebido_em TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,

    sessao_id VARCHAR(64) NOT NULL,
    classe VARCHAR(64) NOT NULL,
    fase ENUM('baseline', 'exposicao', 'recuperacao', 'ambiente', 'indefinida') NOT NULL DEFAULT 'indefinida',
    origem VARCHAR(96) NULL,
    operador VARCHAR(96) NULL,
    observacao TEXT NULL,

    temperatura DOUBLE NULL,
    pressao DOUBLE NULL,
    umidade DOUBLE NULL,
    gas DOUBLE NULL,
    altitude DOUBLE NULL,

    bsec_iaq DOUBLE NULL,
    bsec_static_iaq DOUBLE NULL,
    bsec_co2_equivalent DOUBLE NULL,
    bsec_breath_voc_equivalent DOUBLE NULL,
    bsec_gas_percentage DOUBLE NULL,
    bsec_compensated_temperature DOUBLE NULL,
    bsec_compensated_humidity DOUBLE NULL,

    heater_profile VARCHAR(96) NULL,
    duty_cycle VARCHAR(96) NULL,
    bsec_config_nome VARCHAR(128) NULL,
    bme_ai_studio_project VARCHAR(128) NULL,

    amostra_valida BOOLEAN NOT NULL DEFAULT TRUE,
    motivo_invalidacao VARCHAR(255) NULL,

    INDEX idx_ia_sessao (sessao_id),
    INDEX idx_ia_classe (classe),
    INDEX idx_ia_fase (fase),
    INDEX idx_ia_recebido_em (recebido_em)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

CREATE TABLE IF NOT EXISTS dados_bme688_ia_inferencias (
    id INT AUTO_INCREMENT PRIMARY KEY,
    inferido_em TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,

    treinamento_id INT NULL,
    sessao_id VARCHAR(64) NULL,

    modelo_nome VARCHAR(128) NOT NULL,
    modelo_versao VARCHAR(64) NULL,
    origem_modelo ENUM('python', 'bsec', 'bsec2', 'embarcado', 'manual') NOT NULL DEFAULT 'python',

    classe_predita VARCHAR(64) NOT NULL,
    classe_real VARCHAR(64) NULL,
    confianca DOUBLE NULL,

    prob_ar_limpo DOUBLE NULL,
    prob_alcool DOUBLE NULL,
    prob_fumaca DOUBLE NULL,
    prob_cafe DOUBLE NULL,
    prob_ambiente_externo DOUBLE NULL,
    prob_outros DOUBLE NULL,

    temperatura DOUBLE NULL,
    pressao DOUBLE NULL,
    umidade DOUBLE NULL,
    gas DOUBLE NULL,
    altitude DOUBLE NULL,

    payload_json JSON NULL,
    observacao TEXT NULL,

    INDEX idx_inf_modelo (modelo_nome),
    INDEX idx_inf_classe_predita (classe_predita),
    INDEX idx_inf_inferido_em (inferido_em),
    INDEX idx_inf_treinamento_id (treinamento_id),

    CONSTRAINT fk_inferencia_treinamento
        FOREIGN KEY (treinamento_id)
        REFERENCES dados_bme688_ia_treinamento(id)
        ON DELETE SET NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

SHOW TABLES;
