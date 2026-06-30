USE IoT;

DESCRIBE dados_bme688_colunar;

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

SELECT COUNT(*) AS total_visualizacao
FROM dados_bme688_colunar;
