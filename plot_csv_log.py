# /// script
# dependencies = [
#   "pandas",
#   "numpy",
#   "matplotlib",
# ]
# ///

"""
AI-GENERATED FILE:
plot_lqr_run.py
---------------
Plots a single LQR inverted pendulum run from a CSV log.

CSV format (no header row, 9 columns):
    t_us, state, Force, x, xdot, phi, phidot, event, extra

State values:
    1 = controller active (LQR running)
    0 = motor off (IDLE or post-crash)

Events expected in the 'event' column:
    'start'               — controller activated
    'crash (limit_switch) — limit switch hit (may be false trigger)

Units:
    t_us   : microseconds
    Force  : raw control output (float)
    x      : meters
    xdot   : m/s
    phi    : radians  (0 = upright, ±π = hanging down)
    phidot : rad/s

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  REPLACE THESE VALUES FOR EACH NEW RUN
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

# ── CONFIG — edit these ────────────────────────────────────────────────────────

CSV_PATH   = "logs/runtime-test-05_20260329_141758.csv"   # path to your CSV log
RUN_LABEL  = "dancing-queen-04"                        # name shown in plot titles
K_GAINS    = [10.0, 50.0, 152.4200, 30.0335]      # K = [K1, K2, K3, K4]

OUTPUT_DIR = "."   # folder to save PNGs (use "." for current directory)

# ── END CONFIG ─────────────────────────────────────────────────────────────────

import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# ── Style ──────────────────────────────────────────────────────────────────────
plt.rcParams.update({
    'font.family': 'monospace', 'font.size': 9.5,
    'axes.titlesize': 10.5, 'axes.titleweight': 'bold',
    'figure.facecolor': '#0d1117', 'axes.facecolor': '#161b22',
    'axes.edgecolor': '#30363d', 'axes.labelcolor': '#c9d1d9',
    'xtick.color': '#8b949e', 'ytick.color': '#8b949e',
    'text.color': '#e6edf3', 'grid.color': '#21262d', 'grid.alpha': 1.0,
    'lines.linewidth': 1.4, 'figure.dpi': 140,
})
CYAN   = '#79c0ff'
GREEN  = '#3fb950'
ORANGE = '#d29922'
RED    = '#f85149'
PURPLE = '#bc8cff'
YELLOW = '#e3b341'
WHITE  = '#e6edf3'
GRAY   = '#8b949e'

# ── Load & parse ───────────────────────────────────────────────────────────────
# Auto-detects whether the file has a header row or not.
# Header row is expected to look like: "Time (μs),State,Force (N),..."
# If the first cell is not numeric, it is treated as a header and skipped.

COL_NAMES = ['t_us', 'state', 'Force', 'x', 'xdot', 'phi', 'phidot', 'event', 'extra']

# Find the first data line, skipping PlatformIO preamble (lines starting with '---')
with open(CSV_PATH, 'r') as f:
    lines = f.readlines()
skip = 0
for line in lines:
    stripped = line.strip()
    if stripped.startswith('---') or stripped == '':
        skip += 1
    else:
        break
first_cell = lines[skip].split(',')[0].strip() if skip < len(lines) else ''

has_header = not first_cell.lstrip('-').replace('.', '', 1).isdigit()
df = pd.read_csv(CSV_PATH, header=0 if has_header else None,
                 skiprows=skip if not has_header else skip,
                 names=None if has_header else COL_NAMES)

if has_header:
    # Rename whatever the actual header says to our standard names
    df.columns = COL_NAMES[:len(df.columns)]

for c in ['t_us', 'state', 'Force', 'x', 'xdot', 'phi', 'phidot']:
    df[c] = pd.to_numeric(df[c], errors='coerce')
df['event'] = df['event'].fillna('').astype(str).str.strip()
df = df.dropna(subset=['t_us'])

# ── Split into phases ──────────────────────────────────────────────────────────
ctrl = df[df['state'] == 1].copy()
if ctrl.empty:
    raise ValueError("No State=1 rows found. Check your CSV.")

t0 = ctrl['t_us'].iloc[0]   # time at controller start
ctrl['t_s'] = (ctrl['t_us'] - t0) / 1e6

crash_t_us = ctrl['t_us'].iloc[-1]
post = df[(df['state'] == 0) & (df['t_us'] > crash_t_us)].copy()
post['t_s'] = (post['t_us'] - t0) / 1e6

dur       = ctrl['t_s'].iloc[-1]
phi_max   = ctrl['phi'].abs().max()
phi_rms   = np.sqrt(np.mean(ctrl['phi'] ** 2))
enc_res   = 2 * np.pi / (600 * 4)   # rad per encoder tick (600 CPR, 4x decode)

k_str = f"K=[{', '.join(str(k) for k in K_GAINS)}]"
title_base = f"{RUN_LABEL}  ·  {k_str}  ·  {dur:.1f}s"

def savefig(fig, name):
    path = os.path.join(OUTPUT_DIR, f"{RUN_LABEL}_{name}.png")
    fig.savefig(path, dpi=140, bbox_inches='tight', facecolor=fig.get_facecolor())
    print(f"Saved: {path}")

# ══════════════════════════════════════════════════════════════════════════════
# FIG 1 — Full timeline: φ, x, Force
# ══════════════════════════════════════════════════════════════════════════════
fig1, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
fig1.suptitle(title_base, fontsize=12, fontweight='bold', color=WHITE, y=0.99)

# φ
ax = axes[0]
ax.plot(ctrl['t_s'], ctrl['phi'], color=GREEN, lw=1.0, label='State=1 (LQR active)')
if not post.empty:
    ax.plot(post['t_s'], post['phi'], color=GRAY, lw=1.0, alpha=0.7, label='State=0 (motor off)')
ax.axhline(0,          color=WHITE,  lw=0.7, ls='--', alpha=0.3, label='φ=0 (upright)')
ax.axhline( np.pi/2,   color=ORANGE, lw=0.6, ls=':',  alpha=0.5, label='±π/2')
ax.axhline(-np.pi/2,   color=ORANGE, lw=0.6, ls=':',  alpha=0.5)
ax.axhline( np.pi,     color=RED,    lw=0.6, ls=':',  alpha=0.4, label='±π (hanging)')
ax.axhline(-np.pi,     color=RED,    lw=0.6, ls=':',  alpha=0.4)
ax.axvline(dur, color=RED, lw=1.5, ls='--', alpha=0.8, label='End of State=1')
ax.set_ylabel('φ  (radians)')
ax.set_title('Pendulum Angle', loc='left')
ax.legend(fontsize=7.5, loc='lower left', framealpha=0.2, ncol=3)
ax.text(0.99, 0.97, f'|φ|_max = {phi_max:.4f} rad  ({phi_max*180/np.pi:.2f}°)   RMS = {phi_rms:.4f} rad',
        transform=ax.transAxes, ha='right', va='top', fontsize=8.5, color=GREEN,
        bbox=dict(facecolor='#0d1117', alpha=0.8, edgecolor=GREEN, lw=0.7))
ax.grid(True, lw=0.5)

# x
ax = axes[1]
ax.plot(ctrl['t_s'], ctrl['x'], color=CYAN, lw=1.2, label='State=1')
if not post.empty:
    ax.plot(post['t_s'], post['x'], color=GRAY, lw=1.0, alpha=0.7, label='State=0')
ax.axhline(0, color=WHITE, lw=0.6, ls='--', alpha=0.25)
ax.axvline(dur, color=RED, lw=1.5, ls='--', alpha=0.8)
ax.set_ylabel('x  (m)')
ax.set_title('Cart Position', loc='left')
ax.legend(fontsize=8, loc='upper right', framealpha=0.2)
ax.grid(True, lw=0.5)

# Force
ax = axes[2]
ax.plot(ctrl['t_s'], ctrl['Force'], color=PURPLE, lw=0.9, alpha=0.9, label='State=1')
if not post.empty:
    ax.plot(post['t_s'], post['Force'], color=GRAY, lw=0.8, alpha=0.5, label='State=0')
ax.axhline(0, color=WHITE, lw=0.5, ls='--', alpha=0.2)
ax.axvline(dur, color=RED, lw=1.5, ls='--', alpha=0.8)
ax.set_ylabel('Force  (N)')
ax.set_xlabel('Time  (s)  [from controller start]')
ax.set_title('Control Output', loc='left')
ax.legend(fontsize=8, loc='upper right', framealpha=0.2)
ax.grid(True, lw=0.5)

for ax in axes:
    ax.tick_params(labelsize=8.5)
plt.tight_layout(rect=[0, 0, 1, 0.98])
savefig(fig1, 'fig1_overview')

# ══════════════════════════════════════════════════════════════════════════════
# FIG 2 — φ zoom (±π/2 scale) + phase portrait, State=1 only
# ══════════════════════════════════════════════════════════════════════════════
fig2, axes2 = plt.subplots(1, 2, figsize=(14, 6))
fig2.suptitle(f'{RUN_LABEL}  ·  State=1 Detail', fontsize=12,
              fontweight='bold', color=WHITE, y=1.01)

# φ(t) zoomed
ax = axes2[0]
ax.plot(ctrl['t_s'], ctrl['phi'], color=GREEN, lw=1.0)
ax.axhline(0,         color=WHITE,  lw=0.7, ls='--', alpha=0.3, label='φ=0')
ax.axhline( np.pi/2,  color=ORANGE, lw=0.6, ls=':',  alpha=0.5, label='±π/2')
ax.axhline(-np.pi/2,  color=ORANGE, lw=0.6, ls=':',  alpha=0.5)
ax.axhline( enc_res,  color=YELLOW, lw=0.5, ls=':',  alpha=0.5,
            label=f'±1 tick = ±{enc_res:.4f} rad')
ax.axhline(-enc_res,  color=YELLOW, lw=0.5, ls=':',  alpha=0.5)
ax.set_ylim(-np.pi/2 - 0.1, np.pi/2 + 0.1)
ax.set_xlabel('Time (s)')
ax.set_ylabel('φ  (radians)')
ax.set_title('φ(t) — full ±π/2 scale for reference', loc='left')
ax.legend(fontsize=8, framealpha=0.2)
ax.text(0.99, 0.97, f'RMS = {phi_rms:.4f} rad  ({phi_rms*180/np.pi:.2f}°)',
        transform=ax.transAxes, ha='right', va='top', fontsize=9, color=GREEN,
        bbox=dict(facecolor='#0d1117', alpha=0.8, edgecolor=GREEN, lw=0.7))
ax.grid(True, lw=0.5)
ax.set_facecolor('#161b22')

# Phase portrait
ax = axes2[1]
sc = ax.scatter(ctrl['phi'], ctrl['phidot'], c=ctrl['t_s'],
                cmap='plasma', s=6, zorder=3, alpha=0.8)
ax.plot(ctrl['phi'], ctrl['phidot'], color=WHITE, lw=0.3, alpha=0.15, zorder=2)
cbar = plt.colorbar(sc, ax=ax)
cbar.set_label('Time (s)', color=WHITE, fontsize=8)
cbar.ax.yaxis.set_tick_params(color=WHITE, labelsize=7)
plt.setp(cbar.ax.yaxis.get_ticklabels(), color=WHITE)
ax.axhline(0, color=WHITE, lw=0.5, ls='--', alpha=0.2)
ax.axvline(0, color=WHITE, lw=0.5, ls='--', alpha=0.2)
ax.scatter([ctrl['phi'].iloc[0]],  [ctrl['phidot'].iloc[0]],
           color=GREEN, s=80, zorder=5, marker='*', label='Start')
ax.scatter([ctrl['phi'].iloc[-1]], [ctrl['phidot'].iloc[-1]],
           color=RED,   s=80, zorder=5, marker='X', label='End')
ax.set_xlabel('φ  (radians)')
ax.set_ylabel('φ̇  (rad/s)')
ax.set_title('Phase Portrait  (φ, φ̇)\nTight orbit = stable', loc='left')
ax.legend(fontsize=8.5, framealpha=0.2)
ax.grid(True, lw=0.5)
ax.set_facecolor('#161b22')

for ax in axes2:
    ax.tick_params(labelsize=8.5)
plt.tight_layout()
savefig(fig2, 'fig2_detail')

# ══════════════════════════════════════════════════════════════════════════════
# FIG 3 — Post-crash free swing (State=0 after limit switch)
#          Only generated if there is post-crash data
# ══════════════════════════════════════════════════════════════════════════════
if not post.empty:
    fig3, ax3 = plt.subplots(figsize=(12, 5))
    fig3.suptitle(f'{RUN_LABEL}  ·  Post-crash Free Swing (State=0)',
                  fontsize=12, fontweight='bold', color=WHITE)
    ax3.plot(post['t_s'], post['phi'], color=GRAY, lw=1.2)
    ax3.axhline(0,        color=WHITE,  lw=0.6, ls='--', alpha=0.3, label='φ=0 (upright)')
    ax3.axhline( np.pi,   color=RED,    lw=0.6, ls=':',  alpha=0.5, label='±π (hanging)')
    ax3.axhline(-np.pi,   color=RED,    lw=0.6, ls=':',  alpha=0.5)
    ax3.axhline( np.pi/2, color=ORANGE, lw=0.5, ls=':',  alpha=0.4, label='±π/2')
    ax3.axhline(-np.pi/2, color=ORANGE, lw=0.5, ls=':',  alpha=0.4)
    ax3.set_xlabel('Time (s) from controller start')
    ax3.set_ylabel('φ  (radians)')
    ax3.set_title('Pendulum swinging freely after controller stopped', loc='left')
    ax3.legend(fontsize=8.5, framealpha=0.2)
    ax3.grid(True, lw=0.5)
    ax3.tick_params(labelsize=8.5)
    plt.tight_layout()
    savefig(fig3, 'fig3_freeswing')

plt.show()
print("Done.")
