USE IoT;

DESCRIBE dados_bme688_ia_inferencias;

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

SELECT COUNT(*) AS total_inferencias
FROM dados_bme688_ia_inferencias;

SELECT
    modelo_nome,
    modelo_versao,
    origem_modelo,
    COUNT(*) AS total_inferencias,
    AVG(confianca) AS confianca_media
FROM dados_bme688_ia_inferencias
GROUP BY modelo_nome, modelo_versao, origem_modelo
ORDER BY total_inferencias DESC;

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
