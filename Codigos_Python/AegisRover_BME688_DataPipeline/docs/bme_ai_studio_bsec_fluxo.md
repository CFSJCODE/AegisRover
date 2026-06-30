# BME AI-Studio, BSEC e BSEC2

## Papel do BME AI-Studio

O BME AI-Studio é usado para organizar coletas, rotular classes e exportar configurações/modelos associados ao BME688. Ele não substitui a necessidade de coleta experimental controlada.

## Papel do BSEC/BSEC2

BSEC/BSEC2 processa leituras do BME688 e pode fornecer saídas como IAQ, CO2 equivalente, VOC equivalente e estimativas de classes quando uma configuração compatível está carregada.

## Arquivo exportado

Para inferência real baseada em configuração treinada, é necessário um arquivo real exportado pelo BME AI-Studio/BSEC. Sem esse arquivo, o sistema deve continuar registrando leitura bruta e dados rotulados, mas não deve anunciar classificação treinada.

## Campos de rastreabilidade no banco

Use:

- `bme_ai_studio_project` para registrar o nome do projeto.
- `bsec_config_nome` para registrar a configuração aplicada.
- `heater_profile` para registrar o perfil de aquecimento.
- `duty_cycle` para registrar o ciclo de operação.

## Não simular IA

Não registre inferências como BSEC/BSEC2 se não houver modelo ou configuração real carregada. Resultados manuais podem ser gravados com `origem_modelo = 'manual'`, desde que documentados como anotação humana.

## Integração futura

Quando o arquivo exportado estiver disponível:

1. Documente o nome do arquivo e do projeto.
2. Integre a configuração no firmware ou no script Python, conforme a estratégia escolhida.
3. Publique ou grave as saídas BSEC/BSEC2 nos campos correspondentes.
4. Grave cada inferência em `dados_bme688_ia_inferencias`.
