# 03_Navegação_LiDAR

Área dedicada à navegação, ao planejamento de rotas e ao processamento de dados do LiDAR.

## Subpastas

| Pasta | Função |
| --- | --- |
| `NavSys_C` | Sistema `navsys` original em C, usado como referência para planejamento, mapa, custo e navegação autônoma. |

## Uso recomendado

- Use esta pasta para algoritmos de planejamento global/local, mapas de ocupação, custo de trajetória e replanejamento.
- O firmware embarcado final deve ficar em `02_Firmware`; esta pasta deve concentrar referências e implementações de navegação.
