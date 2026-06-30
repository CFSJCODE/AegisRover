# Coleta bruta do BME688

## Objetivo

Coletar amostras do BME688 com a estrutura necessária para gerar `.bmerawdata`. Essa coleta é diferente da coleta de visualização e da coleta genérica de IA.

## Tópico MQTT

```text
puc/iot/bme688/raw
```

## Campos obrigatórios

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

## Sessão, classe e fase

Use `sessao_id` para agrupar uma coleta exportável. Use `classe` e `fase` para manter a rastreabilidade:

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

- Não publique valores fictícios de `sensor_id`.
- Não publique `timestamp_since_poweron` sintético se o firmware não medir esse tempo.
- `timestamp_since_poweron` deve ser estritamente crescente dentro da sessão.
- `scanning_cycle_index` deve estar presente.
- Rejeições do coletor aparecem como `[RAW-REJEITADO]` no terminal.

## Limite do BME688 único

Um único BME688 pode gerar dados úteis para estudo, mas o BME AI-Studio foi pensado para fluxos Bosch com metadados específicos e, frequentemente, kits com múltiplos sensores. Por isso, valide o `.bmerawdata` no BME AI-Studio antes de tratar o fluxo como definitivo.
