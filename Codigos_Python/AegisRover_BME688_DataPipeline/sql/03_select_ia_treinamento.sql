USE IoT;

DESCRIBE dados_bme688_ia_treinamento;

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

SELECT COUNT(*) AS total_ia_treinamento
FROM dados_bme688_ia_treinamento;

SELECT
    classe,
    COUNT(*) AS total
FROM dados_bme688_ia_treinamento
GROUP BY classe
ORDER BY total DESC;

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
