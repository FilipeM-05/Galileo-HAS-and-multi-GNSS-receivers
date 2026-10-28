# Galileo HAS and Multi-GNSS Kinematic Simulator

Simulação cinemática e estocástica desenvolvida em Python para o caso de estudo da unidade curricular de **Aviónica e Espaciónica**.

O projeto ilustra o impacto operacional do serviço de alta precisão **Galileo HAS (High Accuracy Service)** e avalia a resiliência e integridade fornecidas por recetores **Multi-GNSS** durante a missão de um Veículo Aéreo Não Tripulado (UAV).

---

## 📌 Contexto do Estudo

A navegação GNSS padrão baseada em medições de pseudodistância por código (SPP) apresenta incertezas nominais de 2 a 3 metros. Em contraste, o **Galileo HAS** fornece correções orbitais, de relógio e enviesamentos diretamente no sinal de satélite (**banda E6-B**) e via **NTRIP**, permitindo posicionamento preciso de ponto (**PPP - Precise Point Positioning**) à escala global sem dependência de estações-base locais (RTK).

Este simulador reproduz numericamente:
1. O **tempo de convergência** e a **precisão sub-decimétrica (< 20 cm)** do Galileo HAS.
2. A **vantagem de redundância e integridade** de uma arquitetura Multi-GNSS perante eventos de perda de sinal ou interferência deliberada (*jamming*) na constelação GPS.

---

## 🚀 Funcionalidades da Simulação

A simulação acompanha uma trajetória 2D de um UAV durante 15 minutos (900 s) e apresenta um painel sincronizado composto por:

- **Trajetória 2D (Plano Horizontal):** Comparação em tempo real entre a rota teórica (*Ground Truth*), um recetor monofrequência GPS convencional e um recetor Multi-GNSS com correções Galileo HAS.
- **Evolução do Erro Horizontal:** Curva de convergência PPP do HAS ao longo dos primeiros 5 minutos até estabilizar abaixo dos 20 cm, em contraste com o ruído contínuo do GPS.
- **Rastreio de Constelações e Fallback:** Monitorização do número de satélites visíveis. Ao minuto 11 (t = 660 s), introduz-se uma falha catastrófica de sinal GPS; o sistema demonstra a perda de posição do recetor de constelação única e a manutenção contínua de integridade via Galileo no recetor Multi-GNSS.

---

## 🛠️ Tecnologias Utilizadas

- Python 3.9+
- NumPy
- Matplotlib

---

## 📦 Instalação e Execução

### 1. Clonar o repositório
git clone https://github.com/o-teu-utilizador/galileo-has-multignss-sim.git
cd galileo-has-multignss-sim

### 2. Criar e ativar o ambiente virtual (opcional)
No Linux/macOS:
python3 -m venv venv
source venv/bin/activate

No Windows:
python -m venv venv
venv\Scripts\activate

### 3. Instalar as dependências
pip install numpy matplotlib

### 4. Executar a simulação
python sim_navigation.py

---

## 📊 Estrutura dos Resultados

Ao correr o script, o sistema gera uma animação interativa com três vistas:

- **Painel Esquerdo (Trajetória 2D Norte-Este):** Precisão de seguimento de rota e desvio pós-falha.
- **Painel Superior Direito (Erro Horizontal vs. Tempo):** Decaimento exponencial da convergência PPP (< 20 cm).
- **Painel Inferior Direito (Satélites Visíveis & Estado):** Efeito de geometria e redundância operacional (*Fallback*).

---

## 📚 Referências

- European Union Agency for the Space Programme (EUSPA). Galileo High Accuracy Service (HAS) Service Definition Document (SDD).
- Kaplan, E. D., & Hegarty, C. J. Understanding GPS/GNSS: Principles and Applications. Artech House.
