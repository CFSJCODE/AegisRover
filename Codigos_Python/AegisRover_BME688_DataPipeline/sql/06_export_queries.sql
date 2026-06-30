USE IoT;

-- Execute esta consulta e use "Export Resultset" no MySQL Workbench para CSV.
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

-- Comandos destrutivos documentados. Execute somente com intencao explicita.
-- TRUNCATE TABLE dados_bme688_colunar;
-- DELETE FROM dados_bme688_ia_inferencias;
-- DELETE FROM dados_bme688_ia_treinamento;
