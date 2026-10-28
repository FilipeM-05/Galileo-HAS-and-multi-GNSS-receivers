"""
Geração de dados da simulação GNSS (GPS standalone vs. Multi-GNSS + Galileo HAS).

Cenário: voo de UAV de 900 s com jamming total do GPS a partir de t = 660 s.
"""

import numpy as np

# --- Parâmetros do cenário ---------------------------------------------------
T_TOTAL = 900          # duração do voo [s]
DT = 1.0               # passo temporal [s]
T_JAM = 660            # instante de início do jamming GPS [s]

# Trajetória (figura em "8" - lemniscata de Gerono)
TRAJ_AX = 500.0        # semi-amplitude em x [m]
TRAJ_AY = 250.0        # semi-amplitude em y [m]

# Satélites
N_GPS_NOMINAL = 8
N_GPS_JAMMED = 2
N_GALILEO = 7

# Erro GPS
GPS_SIGMA = 2.5        # desvio padrão do ruído [m]
GPS_DRIFT_RATE = 10.0  # crescimento linear do erro após jamming [m/s]

# Erro HAS (convergência PPP)
HAS_ERR_INIT = 2.5     # erro inicial [m]
HAS_ERR_FINAL = 0.15   # erro após convergência [m]
HAS_TAU = 60.0         # constante de tempo: exp(-300/60) ≈ 0.7% -> convergido aos 300 s
HAS_NOISE_SIGMA = 0.03 # ruído branco residual [m]

# Direção do erro (passeio aleatório do ângulo para um rasto suave)
HEADING_WALK_SIGMA = 0.15  # [rad/s]

# Geometria dos satélites (para o cálculo do DOP)
SAT_EL_MIN = 15.0          # máscara de elevação [deg]
SAT_EL_MAX = 75.0          # elevação inicial máxima [deg]
SAT_EL_SWING = 5.0         # amplitude da variação de elevação durante o voo [deg]
PERIOD_GPS = 43082.0       # período orbital GPS (~11 h 58 min) [s]
PERIOD_GALILEO = 50680.0   # período orbital Galileo (~14 h 05 min) [s]


def make_time():
    """Vetor de tempo de 0 a T_TOTAL inclusive."""
    return np.arange(0, T_TOTAL + DT, DT)


def make_ground_truth(t):
    """Trajetória ideal em "8": uma volta completa durante o voo."""
    omega = 2 * np.pi / T_TOTAL
    x = TRAJ_AX * np.sin(omega * t)
    y = TRAJ_AY * np.sin(2 * omega * t)
    return x, y


def make_satellites(t):
    """Número de satélites visíveis por constelação."""
    sats_gps = np.where(t < T_JAM, N_GPS_NOMINAL, N_GPS_JAMMED).astype(int)
    sats_galileo = np.full(t.shape, N_GALILEO, dtype=int)
    return sats_gps, sats_galileo


def make_gps_error(t, rng):
    """Erro GPS: |ruído gaussiano| + drift linear sem limite após o jamming."""
    noise = np.abs(rng.normal(0.0, GPS_SIGMA, t.shape))
    drift = GPS_DRIFT_RATE * np.clip(t - T_JAM, 0.0, None)
    return noise + drift


def make_has_error(t, rng):
    """Erro HAS: decaimento exponencial (convergência PPP) + ruído branco suave."""
    decay = HAS_ERR_FINAL + (HAS_ERR_INIT - HAS_ERR_FINAL) * np.exp(-t / HAS_TAU)
    noise = rng.normal(0.0, HAS_NOISE_SIGMA, t.shape)
    return np.clip(decay + noise, 0.0, None)


def make_error_heading(t, rng, freeze_at=None):
    """Ângulo da direção do erro (passeio aleatório).

    Se `freeze_at` for dado, a direção fica fixa a partir desse instante,
    para que o drift se afaste consistentemente da rota.
    """
    steps = rng.normal(0.0, HEADING_WALK_SIGMA, t.shape)
    heading = rng.uniform(0, 2 * np.pi) + np.cumsum(steps)
    if freeze_at is not None:
        idx = np.searchsorted(t, freeze_at)
        heading[idx:] = heading[idx]
    return heading


def apply_error(x_true, y_true, err, heading):
    """Desloca a posição ideal pelo módulo do erro na direção indicada."""
    return x_true + err * np.cos(heading), y_true + err * np.sin(heading)


def make_sky(t, n_sats, period, rng):
    """Azimute e elevação [rad] de n satélites, com movimento orbital lento (T x n).

    Os azimutes iniciais são espalhados pelo céu (com jitter) para uma geometria realista.
    """
    az0 = 2 * np.pi * np.arange(n_sats) / n_sats + rng.uniform(-0.4, 0.4, n_sats)
    el0 = np.radians(rng.uniform(SAT_EL_MIN, SAT_EL_MAX, n_sats))
    phase = rng.uniform(0, 2 * np.pi, n_sats)
    w = 2 * np.pi / period
    az = az0 + w * t[:, None]
    el = el0 + np.radians(SAT_EL_SWING) * np.sin(w * t[:, None] + phase)
    return az, el


def line_of_sight(az, el):
    """Vetores unitários recetor->satélite no referencial local ENU (T x n x 3)."""
    return np.stack([np.cos(el) * np.sin(az),
                     np.cos(el) * np.cos(az),
                     np.sin(el)], axis=-1)


def compute_hdop(blocks):
    """HDOP em cada instante, a partir da matriz de geometria G.

    `blocks` é uma lista (uma entrada por constelação) de pares (los[T,n,3], usado[T,n]).
    Incógnitas: posição (E, N, U) + um enviesamento de relógio por constelação ativa.
    HDOP = sqrt(Q_EE + Q_NN), com Q = (G^T G)^-1. Se houver menos satélites do que
    incógnitas não há solução e o HDOP fica NaN.
    """
    n_t = blocks[0][0].shape[0]
    hdop = np.full(n_t, np.nan)
    for k in range(n_t):
        active = [los[k][used[k]] for los, used in blocks if used[k].any()]
        n_unknowns = 3 + len(active)
        rows = []
        for j, u in enumerate(active):
            clock = np.zeros((len(u), len(active)))
            clock[:, j] = 1.0
            rows.append(np.hstack([-u, clock]))
        g = np.vstack(rows)
        if len(g) < n_unknowns:
            continue
        q = np.linalg.inv(g.T @ g)
        hdop[k] = np.sqrt(q[0, 0] + q[1, 1])
    return hdop


def make_dop(t, sats_gps, sats_galileo, rng):
    """HDOP do recetor GPS standalone e do recetor Multi-GNSS (GPS + Galileo)."""
    los_gps = line_of_sight(*make_sky(t, N_GPS_NOMINAL, PERIOD_GPS, rng))
    los_gal = line_of_sight(*make_sky(t, N_GALILEO, PERIOD_GALILEO, rng))
    # Satélites usados: durante o jamming só os primeiros sats_gps continuam rastreados
    used_gps = np.arange(N_GPS_NOMINAL)[None, :] < sats_gps[:, None]
    used_gal = np.arange(N_GALILEO)[None, :] < sats_galileo[:, None]
    # O recetor Multi-GNSS exclui o GPS durante o jamming (sinais não fiáveis):
    # passa a resolver só com Galileo (fallback)
    used_gps_multi = used_gps & (t < T_JAM)[:, None]
    hdop_gps = compute_hdop([(los_gps, used_gps)])
    hdop_multi = compute_hdop([(los_gps, used_gps_multi), (los_gal, used_gal)])
    return hdop_gps, hdop_multi


def generate_sim_data(seed=42):
    """Gera todos os arrays da simulação e devolve-os num dicionário."""
    rng = np.random.default_rng(seed)
    rng_geo = np.random.default_rng(seed + 1)  # gerador separado: não altera os erros

    t = make_time()
    x_true, y_true = make_ground_truth(t)
    sats_gps, sats_galileo = make_satellites(t)

    err_gps = make_gps_error(t, rng)
    err_has = make_has_error(t, rng)

    heading_gps = make_error_heading(t, rng, freeze_at=T_JAM)
    heading_has = make_error_heading(t, rng)

    x_gps, y_gps = apply_error(x_true, y_true, err_gps, heading_gps)
    x_has, y_has = apply_error(x_true, y_true, err_has, heading_has)

    hdop_gps, hdop_multi = make_dop(t, sats_gps, sats_galileo, rng_geo)

    return {
        "t": t,
        "x_true": x_true, "y_true": y_true,
        "sats_gps": sats_gps, "sats_galileo": sats_galileo,
        "err_gps": err_gps, "err_has": err_has,
        "x_gps": x_gps, "y_gps": y_gps,
        "x_has": x_has, "y_has": y_has,
        "hdop_gps": hdop_gps, "hdop_multi": hdop_multi,
    }


if __name__ == "__main__":
    data = generate_sim_data()
    for name, arr in data.items():
        print(f"{name:>13}: {arr.shape}")
