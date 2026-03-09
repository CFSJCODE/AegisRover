# 🛡️ Aegis Rover

Protótipo UGV (Unmanned Ground Vehicle) autônomo para segurança e controle ambiental.

---

## Sobre o Projeto

O Aegis Rover é um projeto desenvolvido como trabalho prático da disciplina de Internet of Things (IoT), integrante do 3º período do curso de Engenharia de Computação da PUC Minas - Unidade Coração Eucarístico. O desenvolvimento deste protótipo é supervisionado pelo Professor Júlio César Dillinger Conway.

Desenvolvido sobre a base mecânica Rocket Tank, o Aegis Rover atua como um escudo digital proativo (inspirado na mítica Égide). Ele combina mapeamento espacial a laser (LiDAR), aquisição de dados telemétricos ambientais e atuação responsiva de motores e periféricos para detecção e mitigação de anomalias, como vazamentos de gás ou picos de temperatura.

---

## 📑 Índice

1. [Visão Geral e Fundamentação](#-visão-geral-e-fundamentação)
2. [Arquitetura de Hardware](#-arquitetura-de-hardware)
3. [Arquitetura de Software (FreeRTOS)](#-arquitetura-de-software-freertos)
4. [Sistemas de Mobilidade e Defesa Ativa](#-sistemas-de-mobilidade-e-defesa-ativa)
5. [Interface Web (Dashboards SPA)](#-interface-web-dashboards-spa)
6. [Configuração e Deploy](#-configuração-e-deploy)
7. [Equipe e Autoria](#-equipe-e-autoria)

---

## 🎯 Visão Geral e Fundamentação

O sistema Aegis transcende o simples monitoramento passivo. Diferente de sistemas que apenas registram dados, este robô atua no ambiente físico de forma determinística.

Ele integra telemetria LiDAR por triangulação laser para SLAM (Simultaneous Localization and Mapping) e uma camada de comunicação híbrida (Bluetooth Low Energy e Wi-Fi). A adoção de um sistema operacional de tempo real (FreeRTOS) no microcontrolador garante determinismo absoluto no processamento de sinais e redundância na entrega de dados críticos.

**Por que "Aegis"?** Na mitologia, Aegis (Égide) é o escudo impenetrável. O nome transmite imediatamente o conceito de segurança patrimonial, proteção proativa (mitigação de riscos) e monitoramento contínuo.

---

## 🛠️ Arquitetura de Hardware

### Plataforma Central

- **Microcontrolador:** ESP32-C6 (Arquitetura RISC-V, Single-Core)
- **Driver de Potência:** Futura substituição pela placa RoboCore Vespa (Ponte-H integrada)
- **Base Robótica:** Chassi esteirado Rocket Tank (RoboCore)

### Gerenciamento de Energia e Modificações

Para garantir a integridade dos motores e a autonomia do sistema:

- **Alimentação Atual:** Pack de baterias modificado para operar com duas baterias 18650 (7.4V nominal)
- **Upgrade Planejado:** Futura substituição por um pack 2S2P (2 Séries, 2 Paralelos), visando duplicar a capacidade de corrente e autonomia
- **Regulação de Tensão:** Inclusão de um regulador L7805CV para fornecer 5V estáveis aos motores e eletrônica

### Sensores e Atuadores

- **LiDAR LD14P:** Mapeamento 2D 360º (Telemetria e Evasão de Obstáculos)
- **AHT10:** Telemetria Climática de alta precisão (Temperatura e Umidade Relativa)
- **MQ-02:** Detector de gases inflamáveis e fumaça
- **Atuação Ambiental:** Relé para acionamento de Cooler/Exaustor 12V e Buzzer de Alarme

### 🔌 Pinagem Física Principal (ESP32-C6)

| Dispositivo / Fio | Função / Ponto | GPIO ESP32-C6 | Protocolo |
|---|---|---|---|
| LiDAR Branco | Dados (RX) | GPIO 11 | UART1 RX (230400 bd) |
| LiDAR Vermelho | Motor PWM | GPIO 10 | PWM (H-bridge) |
| AHT10 SCL | I2C Clock | GPIO 7 | I2C |
| AHT10 SDA | I2C Data | GPIO 6 | I2C |
| Relé (Cooler) | Atuador Digital | GPIO 5 | Digital Out |
| MQ-02 A0 | Leitura Analógica | GPIO 3 | ADC1_CH3 |
| Buzzer | Alarme Sonoro | GPIO 20 | Digital Out |
| Motor L (Vespa) | PWM / DIR | GPIO 6 / GPIO 7 | Tração Esquerda |
| Motor R (Vespa) | PWM / DIR | GPIO 4 / GPIO 5 | Tração Direita |

---

## 🧠 Arquitetura de Software (FreeRTOS)

O firmware utiliza princípios de Domain-Driven Design (DDD) e o Padrão Monitor, com acesso seguro às variáveis globais através de Mutexes (SemaphoreHandle_t).

### Distribuição de Tarefas (RTOS Scheduler)

- **vTaskLidar:** Processa pacotes UART, valida CRC8 e gera a grade de ocupação espacial
- **vTaskSensor:** Polling I2C estrito do AHT10 e MQ-02 com avaliação de conforto térmico
- **vTaskDrive:** Controle cinemático concorrente (20ms/ciclo) com lógicas de proteção térmica
- **vTaskNetwork & vTaskBLE:** Gerencia infraestrutura TCP/IP (LwIP) e metadados via GATT Bluetooth
- **vTaskPowerMonitor:** Coloca o UGV em Deep-Sleep automático após inatividade para preservar as baterias 18650

---

## 🛡️ Sistemas de Mobilidade e Defesa Ativa

O firmware incorpora camadas de proteção mecânica e elétrica (específicas para a placa Vespa):

- **TCS Progressivo (Traction Control System):** Monitora o Slip Ratio e reduz o torque até 3V para recuperar aderência
- **Modo Overdrive:** Boost temporário (1.8A Peak) para aclives críticos com cooldown térmico obrigatório
- **PIM (Proteção de Integridade Mecânica):** Rampa de desaceleração adaptativa em declives para proteger a caixa de redução (90:1)
- **Rampa Assíncrona (Slew Rate):** Interpolação temporal para mitigar picos indutivos sem bloquear o processador

---

## 💻 Interface Web (Dashboards SPA)

O robô hospeda um WebServer embutido com interfaces baseadas em Tailwind CSS e Vanilla JS.

- **Command Center:** Status geral com temática "Radar UI"
- **LiDAR Radar:** Renderização em Canvas HTML5 da matriz Polar e Cartesiana
- **Climate Hub:** Visualização termográfica e cálculo do "Índice de Conforto Térmico"
- **System Integrity:** Switches em tempo real para ativar/desativar PIM, TCS e Overdrive

---

## 🚀 Configuração e Deploy

### Pré-requisitos

- **IDE:** VS Code + PlatformIO ou Arduino IDE 2.x
- **Placa:** ESP32-C6 Dev Module (Core v3.0.0+)
- **Bibliotecas:** WiFi, WebServer, ESPmDNS, BLEDevice, Wire, esp_pm

### Como Rodar

1. Clone o repositório
2. Configure as credenciais nas constantes `SSID_STA` e `PASSWORD_STA`
3. Ajuste o IP Estático se necessário (Padrão: 192.168.0.55)
4. Compile e faça o upload para o ESP32-C6
5. Acesse `192.168.0.55` no navegador

---

## 📜 Histórico e Nomenclatura

Inicialmente idealizado como Robô Sentinela, o projeto evoluiu para Aegis Rover. A mudança reflete a transição de um sistema de monitoramento passivo para uma plataforma UGV proativa capaz de intervir no ambiente para proteção patrimonial e mitigação de riscos.

---

## 👨‍💻 Equipe e Autoria

| Campo | Informação |
|---|---|
| **Instituição** | PUC Minas - Unidade Coração Eucarístico |
| **Curso** | Engenharia de Computação (3º Período) |
| **Disciplina** | Internet of Things (IoT) |
| **Professor Supervisor** | Júlio César Dillinger Conway |
| **Ano** | 2026 |

### Desenvolvedores

- **Cláudio Francisco Dos Santos Júnior** 
- **Lucas Emanuel Simão Silva**
- **João Pedro Torres**

---

**Construído com componentes RoboCore e tecnologia Espressif.**
