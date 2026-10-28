# Documentação — Simulador Galileo HAS & Multi-GNSS

Documento de referência do simulador: objetivo, cenário, arquitetura do código, modelos matemáticos, funcionamento do dashboard e interpretação dos resultados.

---

## 1. Objetivo

O simulador compara, de forma **cinemática e estocástica** (sem processamento de sinal / SDR real), dois recetores a bordo de um UAV:

| Recetor | Descrição | Comportamento esperado |
|---|---|---|
| **GPS standalone** | Recetor monofrequência, só GPS, posicionamento por código (SPP). | Erro de ~2–3 m; perde a solução quando o GPS é empastelado (*jamming*). |
| **Multi-GNSS + Galileo HAS** | Recetor GPS + Galileo com correções do *High Accuracy Service* (PPP). | Converge para < 20 cm em ~4–5 min; mantém a precisão em *fallback* para Galileo durante o jamming. |

O objetivo é demonstrar visualmente duas propriedades:

1. **Precisão** — a convergência PPP do Galileo HAS até ao nível sub-decimétrico.
2. **Resiliência / integridade** — a redundância de uma arquitetura Multi-GNSS perante a perda de uma constelação.

---

## 2. Cenário simulado

| Parâmetro | Valor |
|---|---|
| Duração do voo | 900 s (15 min), passo de 1 s |
| Trajetória | Figura em "8" (lemniscata de Gerono), 1000 m × 500 m, uma volta completa |
| Evento | **Jamming total do GPS aos 660 s (11:00)** |
| Satélites GPS | 8 antes do jamming → 2 depois |
| Satélites Galileo | 7 constantes (não afetados) |
| Requisito de precisão HAS | < 20 cm |

Linha temporal:

```
0 s ──────────── ~253 s ──────────────────── 660 s ─────────────── 900 s
│  Convergência PPP  │        Nominal            │   Jamming GPS        │
│  (HAS 2.5 → 0.2 m) │  (HAS ≈ 0.15 m, GPS 2.5 m)│ GPS diverge; HAS em  │
│                    │                           │ fallback Galileo     │
```

---

## 3. Arquitetura do código

```
code/
├── data_generator.py   # Modelo: gera todos os dados da simulação (NumPy)
└── sim_navigation.py   # Vista: dashboard Matplotlib, métricas e exportação
outputs/                # PNG / GIF / MP4 gerados (ignorado pelo git)
```

Separação **modelo ↔ vista**: `data_generator.py` não sabe nada de gráficos e devolve um dicionário de arrays; `sim_navigation.py` só lê esse dicionário. Assim é possível alterar o modelo (ex.: outro cenário de jamming) sem tocar no dashboard, e vice-versa.

Fluxo de dados:

```
generate_sim_data(seed)
   ├─ make_time ─────────────► t
   ├─ make_ground_truth ─────► x_true, y_true
   ├─ make_satellites ───────► sats_gps, sats_galileo
   ├─ make_gps_error ────────► err_gps
   ├─ make_has_error ────────► err_has
   ├─ make_error_heading ────► direção do erro (GPS e HAS)
   ├─ apply_error ───────────► x_gps, y_gps, x_has, y_has
   └─ make_dop ──────────────► hdop_gps, hdop_multi
                │
                ▼
sim_navigation.py: compute_stats → build_dashboard → update(frame) → ecrã / ficheiro
```

### Dicionário devolvido por `generate_sim_data()`

| Chave | Unidade | Significado |
|---|---|---|
| `t` | s | Vetor de tempo (0 … 900) |
| `x_true`, `y_true` | m | Posição real (Este, Norte) |
| `x_gps`, `y_gps` | m | Posição estimada pelo recetor GPS |
| `x_has`, `y_has` | m | Posição estimada pelo recetor Multi-GNSS + HAS |
| `err_gps`, `err_has` | m | Módulo do erro horizontal de cada recetor |
| `sats_gps`, `sats_galileo` | — | N.º de satélites rastreados por constelação |
| `hdop_gps`, `hdop_multi` | — | HDOP de cada recetor (`NaN` = sem solução) |

---

## 4. Modelos matemáticos (`data_generator.py`)

### 4.1 Trajetória — `make_ground_truth`

Lemniscata de Gerono, uma volta em `T_TOTAL`:

$$x(t) = A_x \sin(\omega t), \qquad y(t) = A_y \sin(2\omega t), \qquad \omega = 2\pi / T_{total}$$

com $A_x = 500$ m e $A_y = 250$ m. A velocidade resultante é de ~3–5 m/s, típica de um UAV pequeno.

### 4.2 Satélites — `make_satellites`

Função degrau: GPS = 8 para $t < 660$ s e 2 depois; Galileo = 7 sempre. Representa um jammer de banda L1/L2 GPS que não afeta as frequências Galileo usadas (E1/E5/E6).

### 4.3 Erro GPS — `make_gps_error`

$$e_{GPS}(t) = |\mathcal{N}(0,\ 2.5^2)| + 10 \cdot \max(0,\ t - 660)$$

- Antes do jamming: ruído gaussiano com σ = 2.5 m (precisão típica de SPP).
- Depois: **deriva linear sem limite** (10 m/s) — o recetor perdeu a solução e a estimativa diverge (ex.: propagação por um filtro sem medições). Ao fim de 900 s o erro é ≈ 2400 m.

### 4.4 Erro HAS — `make_has_error`

Convergência PPP modelada por um decaimento exponencial + ruído branco:

$$e_{HAS}(t) = e_\infty + (e_0 - e_\infty)\, e^{-t/\tau} + \mathcal{N}(0,\ 0.03^2)$$

| Parâmetro | Valor | Significado |
|---|---|---|
| $e_0$ | 2.5 m | Erro inicial (igual a SPP, antes de haver correções) |
| $e_\infty$ | 0.15 m | Erro após convergência |
| $\tau$ | 60 s | Constante de tempo ($e^{-300/60} \approx 0.7\%$ → convergido em ~5 min) |

O erro HAS **não depende do jamming**: o recetor Multi-GNSS continua a resolver com Galileo + correções HAS (transmitidas no sinal Galileo E6-B).

### 4.5 Direção do erro — `make_error_heading` e `apply_error`

O erro é um escalar; para o desenhar no plano é preciso uma direção. Usa-se um **passeio aleatório do ângulo** (σ = 0.15 rad/s), o que dá rastos suaves em vez de "nuvens" de pontos:

$$x_{est} = x_{true} + e(t)\cos\theta(t), \qquad y_{est} = y_{true} + e(t)\sin\theta(t)$$

Para o GPS, a direção **congela aos 660 s** (`freeze_at=T_JAM`), de modo que a deriva se afasta consistentemente da rota.

### 4.6 Geometria e DOP — `make_sky`, `line_of_sight`, `compute_hdop`, `make_dop`

O **DOP (Dilution of Precision)** mede o quanto a geometria dos satélites amplifica o erro de medição: erro de posição ≈ DOP × erro de pseudodistância. O **HDOP** é a componente horizontal.

1. **`make_sky`** — gera azimute e elevação de cada satélite. Os azimutes iniciais são espalhados pelo céu (com jitter aleatório), as elevações estão entre 15° (máscara) e 75°, e o movimento segue o período orbital real (GPS ≈ 11 h 58 min, Galileo ≈ 14 h 05 min). Em 15 min os satélites quase não se movem, por isso o HDOP é praticamente constante — o que é realista.
2. **`line_of_sight`** — vetor unitário recetor→satélite no referencial local ENU:
   $\mathbf{u} = [\cos el \sin az,\ \cos el \cos az,\ \sin el]$.
3. **`compute_hdop`** — monta a matriz de geometria $G$, uma linha por satélite:
   $$G_i = [-u_E,\ -u_N,\ -u_U,\ \underbrace{1_{GPS},\ 1_{GAL}}_{\text{relógios}}]$$
   Cada constelação ativa acrescenta uma incógnita de relógio (enviesamento inter-sistema). Então
   $$Q = (G^T G)^{-1}, \qquad HDOP = \sqrt{Q_{EE} + Q_{NN}}$$
   Se houver **menos satélites do que incógnitas**, não existe solução e o HDOP é `NaN`.
4. **`make_dop`** — aplica isto aos dois recetores:
   - **GPS standalone**: 4 incógnitas (E, N, U, relógio). Com 8 satélites → HDOP ≈ 1.4; com 2 satélites → **sem solução** (2 < 4).
   - **Multi-GNSS**: 5 incógnitas (E, N, U, relógio GPS, relógio Galileo). Com 8 + 7 satélites → HDOP ≈ 0.76. Durante o jamming o recetor **exclui o GPS** (sinais não fiáveis sob interferência) e resolve só com Galileo (4 incógnitas, 7 satélites) → HDOP ≈ 1.03.

Os dados de geometria usam um gerador aleatório separado (`seed + 1`) para não alterar a sequência aleatória dos erros.

### 4.7 Reprodutibilidade

Todos os resultados são determinísticos para uma dada `seed` (por omissão 42). Mudar a semente (`--seed`) gera outra realização estocástica do mesmo cenário.

---

## 5. Dashboard (`sim_navigation.py`)

### 5.1 Layout

```
┌──────────────────────────── HUD / barra de estado ─────────────────────────────┐
│ Título │ T+ mm:ss │ FASE │ RECETOR GPS │ RECETOR MULTI-GNSS + HAS              │
├─────────────────────────────────────┬──────────────────────────────────────────┤
│                                     │ Erro horizontal vs. tempo (log)          │
│   Trajetória horizontal (E–N)       ├──────────────────────────────────────────┤
│                                     │ Satélites rastreados                     │
│                 ┌── Lupa ±8 m ──┐   ├──────────────────────────────────────────┤
│ Legenda         └───────────────┘   │ Geometria (HDOP)                         │
└─────────────────────────────────────┴──────────────────────────────────────────┘
```

Os três painéis temporais partilham o eixo x (mm:ss), o cursor vertical do instante atual e os mesmos sombreados de fase, para que um evento se leia na vertical em todos eles.

### 5.2 Linguagem visual

| Elemento | Codificação | Razão |
|---|---|---|
| GPS standalone | Laranja `#d95926`, marcador ● | Mesma cor em todos os painéis — a cor identifica a entidade |
| Galileo / Multi-GNSS + HAS | Azul `#3987e5`, marcador ◆ | Idem |
| Ground truth | Cinza claro, tracejado | Referência neutra, não compete com os recetores |
| Estados (HUD, sombreados) | Verde = bom, amarelo = aviso, vermelho = crítico | Sempre acompanhados de ícone + texto (nunca só cor) |

Tema escuro tipo "cockpit" / *ground control station*, com paleta validada para daltonismo e grelha discreta.

### 5.3 HUD — barra de estado

| Campo | Função |
|---|---|
| **Tempo de missão** | Relógio T+ mm:ss |
| **Fase** | `■ CONVERGÊNCIA PPP` (amarelo) → `● NOMINAL · PPP CONVERGIDO` (verde) → `▲ JAMMING GPS · FALLBACK GALILEO` (vermelho) |
| **Recetor GPS** | `● FIX 3D · n sats · erro` (verde) → `▲ SEM FIX · 2 sats · erro` (vermelho) |
| **Recetor Multi-GNSS + HAS** | `■ A CONVERGIR` (amarelo) → `● PPP HAS` (verde) → `● FALLBACK GALILEO` (verde) |

No modo `--static`, o HUD mostra o **resumo de métricas** em vez do estado instantâneo.

### 5.4 Trajetória horizontal

- Ground truth completo (tracejado) + rastos dos dois recetores até ao instante atual.
- Marcadores da posição atual de cada recetor.
- **✕ "Perda GPS · 11:00"** marca onde o GPS perdeu a solução.
- Quando o GPS sai do mapa, uma **seta na borda** indica a direção e a distância ("GPS a 2 401 m fora do mapa"), em vez de alargar os eixos (o que tornaria a rota ilegível).

### 5.5 Lupa (±8 m centrada no UAV)

À escala do mapa (1000 m), erros de 2.5 m e 0.15 m são invisíveis. A lupa mostra o **erro relativo ao ground truth** (ΔE, ΔN) nos últimos 30 s, com o UAV fixo no centro (+):

- Círculo pontilhado de 2.5 m (precisão GPS típica) e círculo sólido de 0.2 m (requisito HAS).
- Vê-se a nuvem laranja do GPS dispersa e o ponto azul do HAS colado ao centro.
- Após o jamming: aviso "GPS fora da lupa (NNN m)".

### 5.6 Erro horizontal vs. tempo

- **Eixo y logarítmico** (0.02 m → 10 000 m): é a única forma de mostrar no mesmo gráfico os 0.15 m do HAS e os 2400 m do GPS.
- Linha do requisito HAS (20 cm).
- Sombreado amarelo = fase de convergência PPP; sombreado vermelho = jamming.
- Linha "PPP convergido mm:ss" — aparece quando a convergência acontece.
- Valor atual de cada recetor escrito junto à ponta da curva.

### 5.7 Satélites rastreados

Curvas em degrau para GPS, Galileo e total. A linha vermelha no 4 marca o **mínimo para fix 3D** (4 incógnitas: x, y, z, relógio). Aos 11:00 o GPS cai para 2 (abaixo do mínimo) enquanto o Galileo se mantém em 7 (acima do mínimo) — é isto que permite o *fallback*.

### 5.8 Geometria (HDOP)

- Linha de referência em HDOP = 1 (geometria ideal).
- Multi-GNSS (≈ 0.76) tem melhor geometria do que GPS sozinho (≈ 1.41), porque tem mais satélites distribuídos pelo céu.
- No jamming, a curva GPS **termina** (sem solução — aviso "GPS sem solução, 2 sats < 4 incógnitas") e a Multi-GNSS sobe para ≈ 1.03 (só Galileo), mas mantém-se boa.

Este painel liga o painel de satélites ao de erro: explica *porque é que* perder satélites faz perder a posição.

---

## 6. Métricas (`compute_stats`)

Calculadas por fase (convergência, nominal, jamming) e por recetor, impressas na consola:

| Métrica | Definição |
|---|---|
| **RMS** | $\sqrt{\overline{e^2}}$ — erro quadrático médio |
| **CEP95** | Percentil 95 do erro horizontal — raio que contém 95% das posições |
| **Máx** | Erro máximo na fase |
| **HDOP méd.** | Média do HDOP (ou "sem sol." se não houve solução) |
| **Tempo de convergência** | Primeiro instante a partir do qual a média móvel de 10 s do erro HAS fica **sempre** abaixo de 20 cm (ignora cruzamentos pontuais causados pelo ruído) |

Resultados com `seed = 42`:

| Fase | Recetor | RMS [m] | CEP95 [m] | Máx [m] | HDOP méd. |
|---|---|---|---|---|---|
| Convergência (0–04:13) | GPS | 2.35 | 4.49 | 7.28 | 1.41 |
| Convergência (0–04:13) | HAS | 0.93 | 2.06 | 2.50 | 0.76 |
| Nominal (04:13–11:00) | GPS | 2.50 | 4.98 | 7.41 | 1.41 |
| Nominal (04:13–11:00) | HAS | 0.16 | 0.21 | 0.25 | 0.76 |
| Jamming (11:00–15:00) | GPS | 1388.77 | 2286.25 | 2400.97 | sem sol. |
| Jamming (11:00–15:00) | HAS | 0.15 | 0.19 | 0.24 | 1.03 |

**Tempo de convergência HAS: 04:13 (253 s).**

Leitura: o HAS é ~16× mais preciso do que o GPS em operação nominal e mantém essa precisão durante o jamming, enquanto o GPS standalone perde completamente a posição.

---

## 7. Execução

```bash
pip install numpy matplotlib

python code/sim_navigation.py               # animação interativa (janela)
python code/sim_navigation.py --static      # outputs/dashboard_final.png + métricas
python code/sim_navigation.py --save gif    # outputs/dashboard.gif
python code/sim_navigation.py --save mp4    # outputs/dashboard.mp4 (requer ffmpeg; senão faz GIF)
```

| Opção | Por omissão | Efeito |
|---|---|---|
| `--static` | — | Guarda o estado final (t = 900 s) em PNG, com o resumo de métricas no HUD |
| `--save {mp4,gif}` | — | Exporta a animação para `outputs/` |
| `--step N` | 2 | Segundos simulados por frame (maior = animação mais rápida / ficheiro menor) |
| `--fps N` | 25 | Frames por segundo da animação |
| `--seed N` | 42 | Semente aleatória da simulação |

Gerar apenas os dados (sem gráficos): `python code/data_generator.py`.

### Principais funções de `sim_navigation.py`

| Função | Responsabilidade |
|---|---|
| `apply_theme` | Configura o tema escuro global do Matplotlib |
| `convergence_time`, `compute_stats`, `print_stats` | Métricas por fase |
| `build_dashboard` | Cria a figura e todos os artistas (vazios) — chama `_build_hud`, `_build_map`, `_build_zoom`, `_build_error`, `_build_sats`, `_build_dop` |
| `update(a, data, i)` | Atualiza todos os artistas para o instante `i` (usada pela animação, pelo PNG e pela exportação) |
| `_offmap_arrow` | Calcula a posição da seta "fora do mapa" na borda |
| `show_summary_hud` | Substitui o HUD pelo resumo de métricas (modo estático) |
| `main` | Interface de linha de comandos |

Parâmetros de apresentação (requisito HAS, tamanho da lupa, limites dos eixos, cores) estão no topo de `sim_navigation.py`; parâmetros do cenário (tempos, erros, satélites, órbitas) no topo de `data_generator.py`.

---

## 8. Simplificações e limitações

O simulador é **ilustrativo**, não um simulador GNSS de alta fidelidade:

- Não há sinais, pseudodistâncias nem filtro de navegação — os erros são gerados diretamente por modelos estocásticos.
- O erro de posição **não é calculado** a partir do HDOP; o DOP é mostrado como indicador de geometria e de disponibilidade de solução.
- A geometria dos satélites é sintética (não usa efemérides/almanaques reais).
- O jamming é um degrau instantâneo; na realidade o C/N₀ degrada-se progressivamente e o número de satélites cai de forma gradual.
- A deriva do GPS após a perda de solução é linear (10 m/s), representando uma propagação sem medições.
- A trajetória é 2D (sem altitude) e a convergência PPP é um decaimento exponencial ideal.

## 9. Extensões possíveis

- **Protection Level (HPL) vs. Alert Limit** — indicador de integridade de tipo aeronáutico (RAIM / ARAIM).
- **Ligar o erro ao DOP** — gerar erros de pseudodistância e obter a posição por mínimos quadrados com $G$.
- **Comparação de cenários** — vários instantes/durações de jamming numa grelha de gráficos estáticos.
- **Degradação progressiva** — modelar C/N₀ e perda gradual de satélites durante o jamming.
