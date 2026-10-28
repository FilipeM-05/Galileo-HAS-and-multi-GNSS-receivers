# Contexto: Simulador GNSS (Galileo HAS & Multi-GNSS)

- **Objetivo**: Simulação cinemática/estocástica em Python para Aviónica. Sem processamento SDR real.
- **Cenário**: Voo de UAV de 15 min (900s). Aos t=660s ocorre *jamming* total na constelação GPS.
- **Stack**: Python, NumPy (matemática estocástica), Matplotlib (dashboard animado).
- **Dashboard (3 painéis)**:
  1. **Esquerda (Trajetória 2D)**: Ground truth (ideal) vs. GPS (rasto com desvio que se perde totalmente aos 660s) vs. Multi-GNSS+HAS (converge para a rota ideal e resiste ao jamming).
  2. **Dir. Sup. (Erro vs Tempo)**: GPS com erro ruidoso de ~2.5m que dispara para infinito aos 660s. HAS com decaimento exponencial de ~2.5m para <0.2m nos primeiros 5 min (convergência PPP) e estável após 660s.
  3. **Dir. Inf. (Satélites vs Tempo)**: GPS cai abruptamente de 8 para <4 aos 660s. Galileo mantém-se constante em 7 satélites, permitindo fallback.
