USE IoT;

-- Invalidar amostra especifica.
UPDATE dados_bme688_ia_treinamento
SET
    amostra_valida = FALSE,
    motivo_invalidacao = 'Leitura instavel durante transicao de exposicao'
WHERE id = 1;

-- Corrigir classe de uma sessao.
UPDATE dados_bme688_ia_treinamento
SET classe = 'alcool'
WHERE sessao_id = 'S001';

-- Corrigir fase de uma faixa de amostras.
UPDATE dados_bme688_ia_treinamento
SET fase = 'baseline'
WHERE sessao_id = 'S001'
  AND recebido_em BETWEEN '2026-06-20 14:00:00' AND '2026-06-20 14:03:00';

-- Exemplo seguro para remover apenas uma sessao de treinamento.
-- As inferencias vinculadas passam a treinamento_id NULL por ON DELETE SET NULL.
DELETE FROM dados_bme688_ia_treinamento
WHERE sessao_id = 'S001';
