# 03_Navegacao_LiDAR

Area dedicada a navegacao, planejamento de rota e processamento de dados do LiDAR.

## Subpastas

| Pasta | Funcao |
| --- | --- |
| `NavSys_C` | Sistema `navsys` original em C, usado como referencia para planejamento, mapa, custo e navegacao autonoma. |

## Uso recomendado

- Use esta pasta para algoritmos de planejamento global/local, mapas de ocupacao, custo de trajetoria e replanejamento.
- Firmware embarcado final deve ficar em `02_Firmware`; esta pasta deve concentrar referencias e implementacoes de navegacao.
