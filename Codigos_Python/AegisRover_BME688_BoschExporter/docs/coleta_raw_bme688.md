# Coleta Raw BME688

## Objetivo

Coletar amostras do BME688 com a estrutura necessaria para gerar `.bmerawdata`. Essa coleta e diferente da coleta de visualizacao e da coleta IA generica.

## Topico MQTT

```text
puc/iot/bme688/raw
```

## Campos obrigatorios

O firmware deve publicar um JSON completo com:

- `sessao_id`
- `sensor_index`
- `sensor_id`
- `timestamp_since_poweron`
- `real_time_clock`
- `temperature`
- `pressure`
- `relative_humidity`
- `resistance_gassensor`
- `heater_profile_step_index`
- `scanning_enabled`
- `scanning_cycle_index`
- `label_tag`
- `error_code`
- `heater_profile_id`
- `duty_cycle_profile_id`
- `board_type`
- `board_id`

Campos opcionais:

- `classe`
- `fase`
- `firmware_version`

## Sessao, classe e fase

Use `sessao_id` para agrupar uma coleta exportavel. Use `classe` e `fase` para manter rastreabilidade:

```json
{
  "sessao_id": "S001",
  "classe": "ar_limpo",
  "fase": "baseline"
}
```

## Heater profile e duty cycle

Perfil inicial:

```json
{
  "heater_profile_id": "heater_354",
  "duty_cycle_profile_id": "duty_1"
}
```

O exportador inicial usa o heater profile `heater_354` com 10 passos e valida `heater_profile_step_index` entre 0 e 9.

## Cuidados

- Nao publique valores ficticios de `sensor_id`.
- Nao publique `timestamp_since_poweron` sintetico se o firmware nao medir esse tempo.
- `timestamp_since_poweron` deve ser estritamente crescente dentro da sessao.
- `scanning_cycle_index` deve estar presente.
- Rejeicoes do coletor aparecem como `[RAW-REJEITADO]` no terminal.

## Limite do BME688 unico

Um unico BME688 pode gerar dados uteis para estudo, mas o BME AI-Studio foi pensado para fluxos Bosch com metadados especificos e, frequentemente, kits com multiplos sensores. Por isso, valide o `.bmerawdata` no BME AI-Studio antes de tratar o fluxo como definitivo.
