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


def generate_sim_data(seed=42):
    """Gera todos os arrays da simulação e devolve-os num dicionário."""
    rng = np.random.default_rng(seed)

    t = make_time()
    x_true, y_true = make_ground_truth(t)
    sats_gps, sats_galileo = make_satellites(t)

    err_gps = make_gps_error(t, rng)
    err_has = make_has_error(t, rng)

    heading_gps = make_error_heading(t, rng, freeze_at=T_JAM)
    heading_has = make_error_heading(t, rng)

    x_gps, y_gps = apply_error(x_true, y_true, err_gps, heading_gps)
    x_has, y_has = apply_error(x_true, y_true, err_has, heading_has)

    return {
        "t": t,
        "x_true": x_true, "y_true": y_true,
        "sats_gps": sats_gps, "sats_galileo": sats_galileo,
        "err_gps": err_gps, "err_has": err_has,
        "x_gps": x_gps, "y_gps": y_gps,
        "x_has": x_has, "y_has": y_has,
    }


if __name__ == "__main__":
    data = generate_sim_data()
    for name, arr in data.items():
        print(f"{name:>13}: {arr.shape}")
