# Coleta de Dados BME688 para IA

## Princípio

A tabela `dados_bme688_colunar` permanece dedicada à visualização e ao monitoramento. A tabela `dados_bme688_ia_treinamento` só recebe amostras quando uma sessão de IA está ativa e uma classe foi definida.

## Sequência recomendada

1. Inicie o script:

```powershell
python bd_bme688_mqtt_ia.py
```

2. Defina a classe:

```text
classe ar_limpo
```

3. Defina a fase:

```text
fase baseline
```

4. Defina a origem:

```text
origem aegis_rover_bancada
```

5. Registre uma observação:

```text
obs coleta de referência antes da exposição
```

6. Inicie a sessão:

```text
sessao iniciar S001
```

7. Verifique o estado:

```text
sessao status
```

8. Encerre a sessão:

```text
sessao parar
```

## Baseline

Use `fase baseline` para registrar o ar de referência antes da exposição ao alvo de classificação. Mantenha o ambiente estável e evite transições bruscas.

## Exposição

Use `fase exposicao` para o período em que o sensor entra em contato com a fonte de interesse, como álcool, fumaça, café ou ambiente externo.

## Recuperação

Use `fase recuperacao` para registrar a volta gradual do sensor ao estado de referência. Essa fase é importante para estudar histerese e tempo de estabilização.

## Cuidados experimentais

- Colete uma classe por sessão sempre que possível.
- Evite misturar conjuntos de treinamento e teste da mesma sessão sem controle.
- Registre origem, operador e observação.
- Aguarde estabilização do BME688 antes de confiar nos dados.
- Prefira o tópico JSON `puc/iot/bme688/telemetria`, pois ele reduz desalinhamento temporal entre variáveis.
- Use os tópicos antigos para compatibilidade e dashboard.
- Marque amostras inválidas no banco em vez de apagá-las quando houver valor científico na rastreabilidade.

## Classes disponíveis

```text
ar_limpo
alcool
fumaca
cafe
ambiente_externo
```

## Fases disponíveis

```text
baseline
exposicao
recuperacao
ambiente
indefinida
```
