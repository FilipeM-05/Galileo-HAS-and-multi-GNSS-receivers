"""
Dashboard animado da simulação GNSS (GPS standalone vs. Multi-GNSS + Galileo HAS).

Painéis:
  - HUD (topo): relógio da missão, fase do voo e estado de cada recetor.
  - Esquerda: trajetória 2D (ground truth, GPS, Multi-GNSS+HAS) + lupa centrada no UAV.
  - Direita superior: erro horizontal vs. tempo (escala logarítmica).
  - Direita inferior: satélites rastreados vs. tempo.

Uso:
  python code/sim_navigation.py               # animação interativa
  python code/sim_navigation.py --static      # PNG do estado final + métricas
  python code/sim_navigation.py --save gif    # exporta a animação (gif ou mp4)
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.gridspec import GridSpec
from matplotlib.patches import Circle
from matplotlib.ticker import FuncFormatter, MultipleLocator, LogLocator

from data_generator import (
    generate_sim_data, T_TOTAL, T_JAM, TRAJ_AX, TRAJ_AY, HAS_ERR_INIT,
)

# --- Parâmetros do dashboard -------------------------------------------------
OUTPUT_DIR = Path(__file__).resolve().parent.parent / "outputs"

HAS_REQUIREMENT = 0.20   # requisito de precisão HAS [m]
CONV_WINDOW = 10         # janela da média móvel para detetar convergência [s]
MIN_SATS_FIX = 4         # mínimo de satélites para fix 3D
ZOOM_HALF = 8.0          # meia-largura da lupa [m]
ZOOM_TRAIL = 30          # segundos de rasto mostrados na lupa
ERR_YLIM = (0.02, 1e4)   # limites do eixo de erro [m]
HDOP_YLIM = (0, 2.5)     # limites do eixo de HDOP

# Limites do mapa: margem extra em baixo para a lupa e a legenda
MAP_XLIM = (-1.12 * TRAJ_AX, 1.12 * TRAJ_AX)
MAP_YLIM = (-3.0 * TRAJ_AY, 1.25 * TRAJ_AY)

# Tema escuro "cockpit" (paleta categórica validada para superfície escura)
THEME = {
    "page": "#0d0d0d",
    "surface": "#1a1a19",
    "hud": "#141413",
    "ink": "#f0efec",
    "ink_2": "#c3c2b7",
    "muted": "#898781",
    "grid": "#2c2c2a",
    "axis": "#383835",
    "gps": "#d95926",       # laranja
    "has": "#3987e5",       # azul (Galileo / Multi-GNSS+HAS)
    "truth": "#c3c2b7",     # cinza claro
    "good": "#0ca30c",
    "warning": "#fab219",
    "critical": "#d03b3b",
}


def apply_theme():
    """Configura o rcParams do Matplotlib com o tema escuro."""
    plt.rcParams.update({
        "figure.facecolor": THEME["page"],
        "axes.facecolor": THEME["surface"],
        "axes.edgecolor": THEME["axis"],
        "axes.labelcolor": THEME["ink_2"],
        "axes.titlecolor": THEME["ink"],
        "axes.titlesize": 12,
        "axes.titleweight": "bold",
        "axes.titlelocation": "left",
        "axes.titlepad": 10,
        "axes.labelsize": 10,
        "axes.grid": True,
        "grid.color": THEME["grid"],
        "grid.linewidth": 0.8,
        "xtick.color": THEME["muted"],
        "ytick.color": THEME["muted"],
        "xtick.labelcolor": THEME["ink_2"],
        "ytick.labelcolor": THEME["ink_2"],
        "xtick.labelsize": 9,
        "ytick.labelsize": 9,
        "legend.facecolor": THEME["surface"],
        "legend.edgecolor": THEME["axis"],
        "legend.labelcolor": THEME["ink"],
        "legend.fontsize": 9,
        "legend.framealpha": 0.92,
        "font.family": "sans-serif",
        "font.sans-serif": ["Segoe UI", "DejaVu Sans", "Arial"],
        "lines.linewidth": 2.0,
        "lines.solid_capstyle": "round",
    })


def fmt_mmss(seconds, _pos=None):
    """Formata segundos como mm:ss."""
    s = int(round(seconds))
    return f"{s // 60:02d}:{s % 60:02d}"


# --- Métricas ----------------------------------------------------------------

def convergence_time(t, err_has):
    """Primeiro instante em que a média móvel do erro HAS fica abaixo do requisito."""
    kernel = np.ones(CONV_WINDOW) / CONV_WINDOW
    smooth = np.convolve(err_has, kernel, mode="same")
    below = smooth < HAS_REQUIREMENT
    # exige que se mantenha abaixo até ao fim (ignora cruzamentos pontuais)
    stays = np.flip(np.logical_and.accumulate(np.flip(below)))
    return float(t[np.argmax(stays)]) if stays.any() else float("nan")


def compute_stats(data):
    """RMS, CEP95 e máximo do erro por fase do voo, para ambos os recetores."""
    t = data["t"]
    t_conv = convergence_time(t, data["err_has"])
    phases = {
        f"Convergência (0–{fmt_mmss(t_conv)})": t < t_conv,
        f"Nominal ({fmt_mmss(t_conv)}–{fmt_mmss(T_JAM)})": (t >= t_conv) & (t < T_JAM),
        f"Jamming ({fmt_mmss(T_JAM)}–{fmt_mmss(T_TOTAL)})": t >= T_JAM,
    }
    stats = {"t_conv": t_conv, "phases": {}}
    for name, mask in phases.items():
        row = {}
        for rx in ("gps", "has"):
            e = data[f"err_{rx}"][mask]
            row[rx] = {
                "rms": float(np.sqrt(np.mean(e ** 2))),
                "cep95": float(np.percentile(e, 95)),
                "max": float(e.max()),
            }
        for rx, key in (("gps", "hdop_gps"), ("has", "hdop_multi")):
            h = data[key][mask]
            row[rx]["hdop"] = float(np.nanmean(h)) if np.isfinite(h).any() else float("nan")
        stats["phases"][name] = row
    return stats


def print_stats(stats):
    """Imprime a tabela de métricas na consola."""
    print(f"\nTempo de convergência HAS (< {HAS_REQUIREMENT * 100:.0f} cm): "
          f"{fmt_mmss(stats['t_conv'])} ({stats['t_conv']:.0f} s)\n")
    header = f"{'Fase':<26}{'Recetor':<10}{'RMS [m]':>12}{'CEP95 [m]':>12}{'Máx [m]':>12}{'HDOP méd.':>12}"
    print(header)
    print("-" * len(header))
    for name, row in stats["phases"].items():
        for rx, label in (("gps", "GPS"), ("has", "HAS")):
            m = row[rx]
            hdop = f"{m['hdop']:.2f}" if np.isfinite(m["hdop"]) else "sem sol."
            print(f"{name:<26}{label:<10}{m['rms']:>12.2f}{m['cep95']:>12.2f}"
                  f"{m['max']:>12.2f}{hdop:>12}")
    print()


# --- Construção da figura ----------------------------------------------------

def _pill(ax, x, y, text="", size=10.5):
    """Etiqueta de estado (texto em tinta + caixa com a cor do estado)."""
    return ax.text(
        x, y, text, transform=ax.transAxes, va="center", ha="left",
        fontsize=size, fontweight="bold", color=THEME["ink"],
        bbox=dict(boxstyle="round,pad=0.45", fc=THEME["surface"],
                  ec=THEME["axis"], lw=1.4),
    )


def _set_pill(pill, text, status):
    """Atualiza texto e cor de estado de uma etiqueta."""
    color = THEME[status]
    pill.set_text(text)
    box = pill.get_bbox_patch()
    box.set_edgecolor(color)
    box.set_facecolor((*plt.matplotlib.colors.to_rgb(color), 0.22))


def _build_hud(fig, gs_cell):
    ax = fig.add_subplot(gs_cell)
    ax.set_facecolor(THEME["hud"])
    ax.set_xticks([])
    ax.set_yticks([])
    ax.grid(False)
    for spine in ax.spines.values():
        spine.set_color(THEME["axis"])

    ax.text(0.012, 0.66, "UAV · MONITOR DE NAVEGAÇÃO GNSS", transform=ax.transAxes,
            fontsize=13, fontweight="bold", color=THEME["ink"], va="center")
    ax.text(0.012, 0.28, "GPS standalone  vs.  Multi-GNSS + Galileo HAS (PPP)",
            transform=ax.transAxes, fontsize=10, color=THEME["ink_2"], va="center")

    labels = {}
    for key, x, text in (("clock", 0.285, "TEMPO DE MISSÃO"),
                         ("phase", 0.425, "FASE"),
                         ("gps", 0.62, "RECETOR GPS"),
                         ("has", 0.78, "RECETOR MULTI-GNSS + HAS")):
        labels[key] = ax.text(x, 0.76, text, transform=ax.transAxes, fontsize=8.5,
                              color=THEME["muted"], fontweight="bold", va="center")

    clock = ax.text(0.285, 0.32, "", transform=ax.transAxes, fontsize=15,
                    fontweight="bold", color=THEME["ink"], va="center",
                    family="monospace")
    return {
        "ax": ax, "labels": labels, "clock": clock,
        "phase": _pill(ax, 0.425, 0.32),
        "gps": _pill(ax, 0.62, 0.32),
        "has": _pill(ax, 0.78, 0.32),
    }


def _build_map(fig, gs_cell, data):
    ax = fig.add_subplot(gs_cell)
    ax.set_title("Trajetória horizontal (Este–Norte)")
    ax.set_xlabel("Este [m]")
    ax.set_ylabel("Norte [m]")
    ax.set_xlim(*MAP_XLIM)
    ax.set_ylim(*MAP_YLIM)
    ax.set_aspect("equal")
    ax.set_anchor("N")

    ax.plot(data["x_true"], data["y_true"], ls=(0, (5, 4)), lw=1.4,
            color=THEME["truth"], alpha=0.55, label="Ground truth")
    a = {
        "ax": ax,
        "gps_trail": ax.plot([], [], color=THEME["gps"], lw=1.6, alpha=0.9,
                             label="GPS standalone")[0],
        "has_trail": ax.plot([], [], color=THEME["has"], lw=2.2,
                             label="Multi-GNSS + HAS")[0],
        "truth_pt": ax.plot([], [], "o", ms=9, mfc=THEME["truth"],
                            mec=THEME["surface"], mew=2, zorder=5)[0],
        "gps_pt": ax.plot([], [], "o", ms=9, mfc=THEME["gps"],
                          mec=THEME["surface"], mew=2, zorder=6)[0],
        "has_pt": ax.plot([], [], "D", ms=8, mfc=THEME["has"],
                          mec=THEME["surface"], mew=2, zorder=7)[0],
    }

    # Marca do instante de perda do GPS
    i_jam = int(np.searchsorted(data["t"], T_JAM))
    xj, yj = data["x_gps"][i_jam], data["y_gps"][i_jam]
    a["jam_mark"] = ax.plot(xj, yj, "X", ms=13, mfc=THEME["critical"],
                            mec=THEME["surface"], mew=1.5, zorder=8)[0]
    a["jam_text"] = ax.annotate(
        f"Perda GPS · {fmt_mmss(T_JAM)}", (xj, yj), xytext=(12, -18),
        textcoords="offset points", fontsize=9, color=THEME["ink"],
        fontweight="bold")

    # Seta na borda quando o GPS sai do mapa
    a["off_arrow"] = ax.annotate(
        "", xy=(0, 0), xytext=(0, 0), textcoords="data",
        arrowprops=dict(arrowstyle="-|>", color=THEME["gps"], lw=2.2,
                        mutation_scale=18))
    a["off_text"] = ax.text(0, 0, "", fontsize=9, color=THEME["ink"],
                            fontweight="bold", ha="center", va="center",
                            bbox=dict(boxstyle="round,pad=0.35", fc=THEME["surface"],
                                      ec=THEME["gps"], lw=1.2))
    a["jam_xy"] = np.array([xj, yj])

    ax.legend(loc="lower left", ncol=1, borderpad=0.8)
    a["zoom"] = _build_zoom(ax)
    return a


def _build_zoom(map_ax):
    """Lupa: erro relativo ao ground truth, centrada na posição do UAV."""
    ax = map_ax.inset_axes([0.685, 0.085, 0.30, 0.32])
    ax.set_facecolor(THEME["hud"])
    ax.set_xlim(-ZOOM_HALF, ZOOM_HALF)
    ax.set_ylim(-ZOOM_HALF, ZOOM_HALF)
    ax.set_aspect("equal")
    ax.xaxis.set_major_locator(MultipleLocator(4))
    ax.yaxis.set_major_locator(MultipleLocator(4))
    ax.tick_params(labelsize=7.5)
    ax.set_title(f"LUPA ±{ZOOM_HALF:.0f} m centrada no UAV (últimos {ZOOM_TRAIL} s)",
                 fontsize=8.5, pad=5, loc="center")
    ax.set_xlabel("ΔE [m]", fontsize=8, labelpad=1)
    ax.set_ylabel("ΔN [m]", fontsize=8, labelpad=1)
    for spine in ax.spines.values():
        spine.set_color(THEME["muted"])

    ax.add_patch(Circle((0, 0), HAS_ERR_INIT, fill=False, ls=":", lw=1,
                        ec=THEME["muted"]))
    ax.text(HAS_ERR_INIT * 0.72, HAS_ERR_INIT * 0.72, f"{HAS_ERR_INIT} m",
            fontsize=7, color=THEME["muted"])
    ax.add_patch(Circle((0, 0), HAS_REQUIREMENT, fill=False, lw=1,
                        ec=THEME["ink_2"]))
    ax.plot(0, 0, "+", ms=12, mew=1.6, color=THEME["truth"], zorder=4)

    return {
        "ax": ax,
        "gps": ax.plot([], [], "-o", lw=0.8, ms=3, color=THEME["gps"], alpha=0.85)[0],
        "has": ax.plot([], [], "-o", lw=0.8, ms=3, color=THEME["has"], zorder=5)[0],
        "gps_now": ax.plot([], [], "o", ms=8, mfc=THEME["gps"],
                           mec=THEME["hud"], mew=1.5, zorder=6)[0],
        "has_now": ax.plot([], [], "D", ms=7, mfc=THEME["has"],
                           mec=THEME["hud"], mew=1.5, zorder=7)[0],
        "note": ax.text(0.5, 0.06, "", transform=ax.transAxes, ha="center",
                        fontsize=8, color=THEME["ink"], fontweight="bold",
                        bbox=dict(boxstyle="round,pad=0.3", fc=THEME["surface"],
                                  ec=THEME["gps"], lw=1)),
    }


def _shade_events(ax, t_conv, label_y):
    """Sombreados de convergência e jamming, comuns aos painéis temporais."""
    ax.axvspan(0, t_conv, color=THEME["warning"], alpha=0.07, lw=0)
    jam = [ax.axvspan(T_JAM, T_TOTAL, color=THEME["critical"], alpha=0.14, lw=0),
           ax.axvline(T_JAM, color=THEME["critical"], lw=1.2, ls="--")]
    texts = []
    if label_y is not None:
        texts.append(ax.text(t_conv / 2, label_y, "▲ CONVERGÊNCIA PPP",
                             transform=ax.get_xaxis_transform(), ha="center",
                             fontsize=8.5, fontweight="bold", color=THEME["ink_2"]))
        texts.append(ax.text((T_JAM + T_TOTAL) / 2, label_y, "▲ JAMMING GPS",
                             transform=ax.get_xaxis_transform(), ha="center",
                             fontsize=8.5, fontweight="bold", color=THEME["ink"]))
    return jam, texts


def _setup_time_axis(ax):
    ax.set_xlim(0, T_TOTAL)
    ax.xaxis.set_major_locator(MultipleLocator(120))
    ax.xaxis.set_minor_locator(MultipleLocator(60))
    ax.xaxis.set_major_formatter(FuncFormatter(fmt_mmss))


def _build_error(fig, gs_cell, data, t_conv):
    ax = fig.add_subplot(gs_cell)
    ax.set_title("Erro horizontal vs. tempo")
    ax.set_ylabel("Erro horizontal [m] (log)")
    ax.set_yscale("log")
    ax.set_ylim(*ERR_YLIM)
    ax.yaxis.set_major_locator(LogLocator(base=10))
    ax.yaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v:g}"))
    _setup_time_axis(ax)
    ax.tick_params(labelbottom=False)

    jam, texts = _shade_events(ax, t_conv, label_y=0.035)
    ax.axhline(HAS_REQUIREMENT, color=THEME["ink_2"], lw=1, ls=(0, (2, 3)),
               label=f"Requisito HAS ({HAS_REQUIREMENT * 100:.0f} cm)")
    conv = [ax.axvline(t_conv, color=THEME["has"], lw=1, ls=":")]
    conv.append(ax.text(t_conv + 6, 40, f"PPP convergido\n{fmt_mmss(t_conv)}",
                        fontsize=8.5, color=THEME["ink_2"], va="center"))

    a = {
        "ax": ax, "jam": jam + texts[1:], "conv": conv,
        "gps": ax.plot([], [], color=THEME["gps"], lw=1.4, label="GPS standalone")[0],
        "has": ax.plot([], [], color=THEME["has"], lw=2.0, label="Multi-GNSS + HAS")[0],
        "gps_pt": ax.plot([], [], "o", ms=8, mfc=THEME["gps"],
                          mec=THEME["surface"], mew=2, zorder=5)[0],
        "has_pt": ax.plot([], [], "D", ms=7, mfc=THEME["has"],
                          mec=THEME["surface"], mew=2, zorder=5)[0],
        "gps_lbl": ax.text(0, 1, "", fontsize=9, fontweight="bold",
                           color=THEME["ink"], va="bottom"),
        "has_lbl": ax.text(0, 1, "", fontsize=9, fontweight="bold",
                           color=THEME["ink"], va="top"),
        "cursor": ax.axvline(0, color=THEME["ink"], lw=1, alpha=0.6),
    }
    ax.legend(loc="upper left", ncol=3)
    return a


def _build_sats(fig, gs_cell, data, t_conv, share_ax):
    ax = fig.add_subplot(gs_cell, sharex=share_ax)
    ax.set_title("Satélites rastreados")
    ax.set_ylabel("N.º de satélites")
    total_max = int((data["sats_gps"] + data["sats_galileo"]).max())
    ax.set_ylim(0, total_max + 5)
    ax.yaxis.set_major_locator(MultipleLocator(4))
    _setup_time_axis(ax)
    ax.tick_params(labelbottom=False)

    jam, _ = _shade_events(ax, t_conv, label_y=None)
    ax.axhline(MIN_SATS_FIX, color=THEME["critical"], lw=1.2, ls=(0, (2, 3)))
    ax.text(8, MIN_SATS_FIX + 0.3, f"Mínimo p/ fix 3D ({MIN_SATS_FIX})",
            fontsize=8.5, color=THEME["ink_2"], va="bottom")

    a = {
        "ax": ax, "jam": jam,
        "total": ax.plot([], [], drawstyle="steps-post", lw=1.6, ls="--",
                         color=THEME["truth"], label="Total (GPS + Galileo)")[0],
        "gps": ax.plot([], [], drawstyle="steps-post", color=THEME["gps"],
                       label="GPS")[0],
        "gal": ax.plot([], [], drawstyle="steps-post", color=THEME["has"],
                       label="Galileo")[0],
        "cursor": ax.axvline(0, color=THEME["ink"], lw=1, alpha=0.6),
    }
    ax.legend(loc="upper left", ncol=3)
    return a


def _build_dop(fig, gs_cell, data, t_conv, share_ax):
    """Painel de geometria: HDOP de cada recetor (NaN = sem solução)."""
    ax = fig.add_subplot(gs_cell, sharex=share_ax)
    ax.set_title("Geometria dos satélites (HDOP)")
    ax.set_ylabel("HDOP")
    ax.set_xlabel("Tempo de missão [mm:ss]")
    ax.set_ylim(*HDOP_YLIM)
    ax.yaxis.set_major_locator(MultipleLocator(0.5))
    _setup_time_axis(ax)

    jam, _ = _shade_events(ax, t_conv, label_y=None)
    ax.axhline(1.0, color=THEME["ink_2"], lw=1, ls=(0, (2, 3)),
               label="HDOP = 1 (geometria ideal)")

    a = {
        "ax": ax, "jam": jam,
        "gps": ax.plot([], [], color=THEME["gps"], label="GPS standalone")[0],
        "multi": ax.plot([], [], color=THEME["has"], label="Multi-GNSS")[0],
        "gps_pt": ax.plot([], [], "o", ms=8, mfc=THEME["gps"],
                          mec=THEME["surface"], mew=2, zorder=5)[0],
        "multi_pt": ax.plot([], [], "D", ms=7, mfc=THEME["has"],
                            mec=THEME["surface"], mew=2, zorder=5)[0],
        "gps_lbl": ax.text(0, 0, "", fontsize=9, fontweight="bold",
                           color=THEME["ink"], va="bottom"),
        "multi_lbl": ax.text(0, 0, "", fontsize=9, fontweight="bold",
                             color=THEME["ink"], va="top"),
        "no_fix": ax.text((T_JAM + T_TOTAL) / 2, 0.80, "▲ GPS sem solução\n2 sats < 4 incógnitas",
                          transform=ax.get_xaxis_transform(), ha="center", va="top",
                          fontsize=8.5, fontweight="bold", color=THEME["ink"],
                          bbox=dict(boxstyle="round,pad=0.3", fc=THEME["surface"],
                                    ec=THEME["critical"], lw=1)),
        "cursor": ax.axvline(0, color=THEME["ink"], lw=1, alpha=0.6),
    }
    ax.legend(loc="upper left", ncol=3)
    return a


def build_dashboard(data, stats):
    """Cria a figura com todos os artistas (vazios) e devolve-os num dicionário."""
    fig = plt.figure(figsize=(16, 9))
    gs = GridSpec(4, 2, figure=fig, height_ratios=[0.11, 0.40, 0.25, 0.25],
                  width_ratios=[1.12, 1], left=0.05, right=0.985, top=0.975,
                  bottom=0.065, hspace=0.42, wspace=0.13)
    t_conv = stats["t_conv"]
    err = _build_error(fig, gs[1, 1], data, t_conv)
    return {
        "fig": fig,
        "hud": _build_hud(fig, gs[0, :]),
        "map": _build_map(fig, gs[1:, 0], data),
        "err": err,
        "sats": _build_sats(fig, gs[2, 1], data, t_conv, err["ax"]),
        "dop": _build_dop(fig, gs[3, 1], data, t_conv, err["ax"]),
        "t_conv": t_conv,
    }


# --- Atualização -------------------------------------------------------------

def _offmap_arrow(m, x, y):
    """Coloca a seta na borda do mapa na direção do GPS (se estiver fora)."""
    (x0, x1), (y0, y1) = MAP_XLIM, MAP_YLIM
    inside = x0 <= x <= x1 and y0 <= y <= y1
    m["off_arrow"].set_visible(not inside)
    m["off_text"].set_visible(not inside)
    if inside:
        return
    c = m["jam_xy"]
    d = np.array([x, y]) - c
    d /= np.linalg.norm(d)
    # distância até à borda (com pequena margem) ao longo de d
    margin = 0.97
    s = min(
        ((x1 - c[0]) if d[0] > 0 else (x0 - c[0])) / d[0] if d[0] else np.inf,
        ((y1 - c[1]) if d[1] > 0 else (y0 - c[1])) / d[1] if d[1] else np.inf,
    ) * margin
    tip = c + s * d
    tail = tip - 90 * d
    m["off_arrow"].xy = tuple(tip)
    m["off_arrow"].set_position(tuple(tail))
    m["off_text"].set_position(tuple(tip - 190 * d))


def update(a, data, i):
    """Atualiza todos os artistas para o índice temporal i."""
    d = data
    t = d["t"][i]
    t_conv = a["t_conv"]
    jammed = t >= T_JAM

    # Mapa
    m = a["map"]
    m["gps_trail"].set_data(d["x_gps"][:i + 1], d["y_gps"][:i + 1])
    m["has_trail"].set_data(d["x_has"][:i + 1], d["y_has"][:i + 1])
    m["truth_pt"].set_data([d["x_true"][i]], [d["y_true"][i]])
    m["gps_pt"].set_data([d["x_gps"][i]], [d["y_gps"][i]])
    m["has_pt"].set_data([d["x_has"][i]], [d["y_has"][i]])
    m["jam_mark"].set_visible(jammed)
    m["jam_text"].set_visible(jammed)
    _offmap_arrow(m, d["x_gps"][i], d["y_gps"][i])
    m["off_text"].set_text(f"GPS a {d['err_gps'][i]:,.0f} m\nfora do mapa".replace(",", " "))

    # Lupa (erro relativo ao ground truth)
    z = m["zoom"]
    j0 = max(0, i - ZOOM_TRAIL)
    dx_g = d["x_gps"][j0:i + 1] - d["x_true"][j0:i + 1]
    dy_g = d["y_gps"][j0:i + 1] - d["y_true"][j0:i + 1]
    dx_h = d["x_has"][j0:i + 1] - d["x_true"][j0:i + 1]
    dy_h = d["y_has"][j0:i + 1] - d["y_true"][j0:i + 1]
    z["gps"].set_data(dx_g, dy_g)
    z["has"].set_data(dx_h, dy_h)
    z["gps_now"].set_data([dx_g[-1]], [dy_g[-1]])
    z["has_now"].set_data([dx_h[-1]], [dy_h[-1]])
    gps_out = d["err_gps"][i] > ZOOM_HALF
    z["note"].set_visible(gps_out)
    z["note"].set_text(f"GPS fora da lupa ({d['err_gps'][i]:.0f} m)")

    # Erro vs tempo
    e = a["err"]
    tt = d["t"][:i + 1]
    eg = np.clip(d["err_gps"][:i + 1], ERR_YLIM[0], None)
    eh = np.clip(d["err_has"][:i + 1], ERR_YLIM[0], None)
    e["gps"].set_data(tt, eg)
    e["has"].set_data(tt, eh)
    e["gps_pt"].set_data([t], [eg[-1]])
    e["has_pt"].set_data([t], [eh[-1]])
    lbl_x = min(t + 10, T_TOTAL - 95)
    e["gps_lbl"].set_position((lbl_x, eg[-1] * 1.35))
    e["gps_lbl"].set_text(f"{eg[-1]:.1f} m" if eg[-1] < 100 else f"{eg[-1]:.0f} m")
    e["has_lbl"].set_position((lbl_x, eh[-1] * 0.75))
    e["has_lbl"].set_text(f"{eh[-1]:.2f} m")
    e["cursor"].set_xdata([t, t])
    for art in e["jam"] + a["sats"]["jam"] + a["dop"]["jam"]:
        art.set_visible(jammed)
    for art in e["conv"]:
        art.set_visible(t >= t_conv)

    # Satélites
    s = a["sats"]
    sg = d["sats_gps"][:i + 1]
    sl = d["sats_galileo"][:i + 1]
    s["gps"].set_data(tt, sg)
    s["gal"].set_data(tt, sl)
    s["total"].set_data(tt, sg + sl)
    s["cursor"].set_xdata([t, t])

    # Geometria (HDOP)
    g = a["dop"]
    hg = d["hdop_gps"][:i + 1]
    hm = d["hdop_multi"][:i + 1]
    g["gps"].set_data(tt, hg)
    g["multi"].set_data(tt, hm)
    g["gps_pt"].set_data([t], [hg[-1]])
    g["multi_pt"].set_data([t], [hm[-1]])
    g["multi_lbl"].set_position((lbl_x, hm[-1] - 0.12))
    g["multi_lbl"].set_text(f"{hm[-1]:.2f}")
    g["gps_lbl"].set_visible(np.isfinite(hg[-1]))
    g["gps_lbl"].set_position((lbl_x, hg[-1] + 0.12 if np.isfinite(hg[-1]) else 0))
    g["gps_lbl"].set_text(f"{hg[-1]:.2f}")
    g["no_fix"].set_visible(not np.isfinite(hg[-1]))
    g["cursor"].set_xdata([t, t])

    # HUD
    h = a["hud"]
    n_gps, n_gal = int(d["sats_gps"][i]), int(d["sats_galileo"][i])
    h["clock"].set_text(f"T+ {fmt_mmss(t)}")
    if jammed:
        _set_pill(h["phase"], "▲ JAMMING GPS · FALLBACK GALILEO", "critical")
        _set_pill(h["gps"], f"▲ SEM FIX · {n_gps} sats · {d['err_gps'][i]:.0f} m", "critical")
        _set_pill(h["has"], f"● FALLBACK GALILEO · {n_gal} sats · {d['err_has'][i]:.2f} m", "good")
    elif t < t_conv:
        _set_pill(h["phase"], "■ CONVERGÊNCIA PPP", "warning")
        _set_pill(h["gps"], f"● FIX 3D · {n_gps} sats · {d['err_gps'][i]:.1f} m", "good")
        _set_pill(h["has"], f"■ A CONVERGIR · {n_gps + n_gal} sats · {d['err_has'][i]:.2f} m", "warning")
    else:
        _set_pill(h["phase"], "● NOMINAL · PPP CONVERGIDO", "good")
        _set_pill(h["gps"], f"● FIX 3D · {n_gps} sats · {d['err_gps'][i]:.1f} m", "good")
        _set_pill(h["has"], f"● PPP HAS · {n_gps + n_gal} sats · {d['err_has'][i]:.2f} m", "good")


def show_summary_hud(a, stats):
    """Substitui o estado instantâneo do HUD pelo resumo de métricas (modo estático)."""
    h = a["hud"]
    phases = list(stats["phases"].values())
    gps_nom = phases[1]["gps"]
    has_post = [p["has"] for p in phases[1:]]
    has_rms = max(p["rms"] for p in has_post)
    has_cep = max(p["cep95"] for p in has_post)
    h["labels"]["phase"].set_text("CONVERGÊNCIA HAS (< 20 cm)")
    h["labels"]["gps"].set_text("GPS · FASE NOMINAL")
    h["labels"]["has"].set_text("HAS · APÓS CONVERGÊNCIA (INCL. JAMMING)")
    _set_pill(h["phase"], f"■ PPP convergido aos {fmt_mmss(stats['t_conv'])}", "good")
    _set_pill(h["gps"], f"RMS {gps_nom['rms']:.2f} m · CEP95 {gps_nom['cep95']:.2f} m", "warning")
    _set_pill(h["has"], f"● RMS {has_rms:.2f} m · CEP95 {has_cep:.2f} m", "good")


# --- Execução ----------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Dashboard GNSS: GPS vs. Multi-GNSS + Galileo HAS")
    parser.add_argument("--static", action="store_true",
                        help="guarda um PNG do estado final em outputs/")
    parser.add_argument("--save", choices=["mp4", "gif"],
                        help="exporta a animação para outputs/")
    parser.add_argument("--step", type=int, default=2,
                        help="segundos simulados por frame (default: 2)")
    parser.add_argument("--fps", type=int, default=25, help="frames por segundo")
    parser.add_argument("--seed", type=int, default=42, help="semente aleatória")
    args = parser.parse_args()
    sys.stdout.reconfigure(encoding="utf-8")

    apply_theme()
    data = generate_sim_data(seed=args.seed)
    stats = compute_stats(data)
    print_stats(stats)
    a = build_dashboard(data, stats)
    last = len(data["t"]) - 1

    if args.static:
        OUTPUT_DIR.mkdir(exist_ok=True)
        update(a, data, last)
        show_summary_hud(a, stats)
        path = OUTPUT_DIR / "dashboard_final.png"
        a["fig"].savefig(path, dpi=150, facecolor=THEME["page"])
        print(f"Figura guardada em {path}")
        return

    frames = list(range(0, last + 1, args.step))
    if frames[-1] != last:
        frames.append(last)
    anim = animation.FuncAnimation(
        a["fig"], lambda i: update(a, data, i), frames=frames,
        interval=1000 / args.fps, blit=False, repeat=args.save is None,
        repeat_delay=2000)

    if args.save:
        OUTPUT_DIR.mkdir(exist_ok=True)
        fmt = args.save
        if fmt == "mp4" and not animation.writers.is_available("ffmpeg"):
            print("ffmpeg não encontrado: a exportar em GIF.")
            fmt = "gif"
        if fmt == "mp4":
            writer, dpi = animation.FFMpegWriter(fps=args.fps, bitrate=4000), 120
        else:
            writer, dpi = animation.PillowWriter(fps=args.fps), 72
        path = OUTPUT_DIR / f"dashboard.{fmt}"
        print(f"A exportar {len(frames)} frames para {path} ...")
        anim.save(path, writer=writer, dpi=dpi, savefig_kwargs={"facecolor": THEME["page"]})
        print("Concluído.")
        return

    plt.show()


if __name__ == "__main__":
    main()
