# AegisRover

O AegisRover é um protótipo UGV acadêmico para IoT, navegação e monitoramento ambiental. O projeto combina firmware ESP32/RoboCore Vespa, LiDAR LD14P, telemetria BME688 via MQTT, persistência MySQL e camadas Python para visualização, treinamento de IA e exportação para o Bosch BME AI-Studio.

## Estado atual

- Sensor ambiental principal: BME688.
- Navegação: LiDAR LD14P com parser, grade de ocupação e módulos C para planejamento.
- Persistência: coletores MQTT/MySQL com tabelas separadas para visualização, treinamento, inferências e exportação Bosch.
- Firmware: sketches Arduino/ESP32 para motores, BME688, LiDAR e integração BME688 + LD14P.
- Referências: datasheets, manuais, limites operacionais e modelos 3D do suporte do LiDAR.

MQ-2 e AHT10 aparecem no histórico do repositório remoto anterior, mas, nesta organização, o fluxo ambiental ativo está centrado no BME688. O LiDAR LD14P permanece como a exceção de navegação.

## Estrutura do repositório

```text
AegisRover/
|-- Banco De Dados/
|   `-- ExibirDados.txt
|-- Codigos_Python/
|   |-- Banco De Dados/
|   |-- AegisRover_BME688_DataPipeline/
|   `-- AegisRover_BME688_BoschExporter/
|-- Datasheets_Manuais/
|-- Firmwares/
|-- Modelo_3D_Suporte_LiDAR/
|-- Sistema_Navegacao_LiDAR/
|-- Softwares/
|-- tools/
`-- .github/workflows/
```

## Pipelines Python

### Pipeline BME688 para IA

```powershell
cd Codigos_Python\AegisRover_BME688_DataPipeline
python -m pip install -r requirements.txt
copy .env.example .env
python bd_bme688_mqtt_ia.py
```

Tabelas principais:

- `dados_bme688_colunar`
- `dados_bme688_ia_treinamento`
- `dados_bme688_ia_inferencias`

Tópicos MQTT aceitos:

```text
puc/iot/bme688/temperatura
puc/iot/bme688/pressao
puc/iot/bme688/umidade
puc/iot/bme688/gas
puc/iot/bme688/altitude
puc/iot/bme688/telemetria
aegis/sensor/bme688/sample
```

### Exportador Bosch BME AI-Studio

```powershell
cd Codigos_Python\AegisRover_BME688_BoschExporter
python -m pip install -r requirements.txt
copy .env.example .env
python collectors\bd_bme688_mqtt_raw.py
python exporters\export_bmerawdata.py --sessao S001 --out exports\bmerawdata
python exporters\validate_bmerawdata.py exports\bmerawdata\S001_ar_limpo.bmerawdata
```

Tabela dedicada:

- `dados_bme688_bosch_raw`

Tópico MQTT raw:

```text
puc/iot/bme688/raw
```

A camada Bosch trabalha em paralelo. Ela não substitui `dados_bme688_colunar`, `dados_bme688_ia_treinamento` nem `dados_bme688_ia_inferencias`.

## Firmware

Arquivos principais:

- `Firmwares/Codigo_BME688_LD14P/Codigo_BME688_LD14P.ino`
- `Firmwares/BME688/BME688_Com_Banco_De_Dados/AegisRover_BME688_MQTT_Adapted/AegisRover_BME688_MQTT_Adapted.ino`
- `Firmwares/CodigoMotores/CodigoMotores.ino`
- `Firmwares/LiDAR/WayPonDEV_LD14P_Wifi/WayPonDEV_LD14P_Wifi.ino`

As bibliotecas Arduino compactadas em `Firmwares/Bibliotecas/` ficam fora do Git. Prefira instalar dependências pela Arduino IDE, pelo PlatformIO ou pelo gerenciador oficial da biblioteca.

## Navegação LiDAR

O núcleo em C fica em `Sistema_Navegacao_LiDAR/NavSys_C` e inclui:

- parser do LD14P;
- filtro de varredura;
- grade de ocupação;
- visualização CSV;
- A*;
- testes de parser, grid, HAL Linux e A*.

Build local dos módulos C:

```powershell
cd Sistema_Navegacao_LiDAR\NavSys_C
bash build.sh --test-parser
bash build.sh --test-grid
bash build.sh --test-astar
bash build.sh --main
```

## Softwares e binários

SDKs Bosch, instaladores, bibliotecas compiladas, arquivos extraídos do BME AI-Studio e outros binários grandes não são versionados. A pasta `Softwares/` contém apenas um README explicando como recompor o ambiente local.

## Validação

Valide o repositório antes de publicar:

```powershell
powershell -ExecutionPolicy Bypass -File tools\validate-project.ps1
```

O script compila todos os arquivos Python. Em Linux/CI, quando `gcc` está disponível, também compila os alvos C principais do `NavSys_C`.

## Autoria

Projeto acadêmico da disciplina Internet das Coisas I, Engenharia de Computação - PUC Minas.
