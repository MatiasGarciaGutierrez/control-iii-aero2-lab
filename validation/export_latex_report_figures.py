"""Create compact report plots from saved MATLAB validation recordings.

Run: uv run --with matplotlib --with scipy python validation/export_latex_report_figures.py
"""
from pathlib import Path
from shutil import copy2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.io import loadmat

ROOT = Path(__file__).resolve().parents[1]
FIGURES = ROOT / 'docs' / 'figures'
REPORT_IMAGES = ROOT / 'report' / 'images'
REPORT_IMAGES.mkdir(parents=True, exist_ok=True)
plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False})

def axes():
    fig, ax = plt.subplots(1, 2, figsize=(9, 2.15), layout='constrained')
    for axis in ax:
        axis.grid(alpha=0.2)
        axis.set_xlabel('Time [s]')
    return fig, ax

pendulum = loadmat(ROOT / 'results' / 'pendulum_regulation_validation.mat', simplify_cells=True)['runs'][0]
fig, ax = axes()
ax[0].plot(pendulum['time'], pendulum['theta'], linewidth=1.8)
for band in [-0.002, 0.002]: ax[0].axhline(band, color='gray', linestyle='--', linewidth=0.8)
ax[0].axvline(0.97232, color='gray', linestyle=':', linewidth=0.8)
ax[0].set_ylabel('Angle [rad]')
ax[1].plot(pendulum['time'], pendulum['u'], linewidth=1.8)
ax[1].set_ylabel('Input [N m]')
fig.savefig(FIGURES / 'report_pendulum_response.png', dpi=200)
copy2(FIGURES / 'report_pendulum_response.png', REPORT_IMAGES / 'report_pendulum_response.png')
plt.close(fig)

recording = loadmat(ROOT / 'results' / 'dc_motor_tracking_validation.mat', simplify_cells=True)
motor = recording['runs'][0]
fig, ax = axes()
ax[0].plot(motor['time'], motor['theta'], linewidth=1.8, label='Angle')
ax[0].plot(motor['time'], motor['reference'], '--', linewidth=1.1, label='Reference')
ax[0].axvline(0.05 + recording['predictedSettling'], color='gray', linestyle=':', linewidth=0.8)
ax[0].set_ylabel('Angle [rad]')
ax[0].legend(frameon=False, fontsize=8)
ax[1].plot(motor['time'], motor['u'], linewidth=1.8)
ax[1].set_ylabel('Voltage [V]')
fig.savefig(FIGURES / 'report_dc_motor_tracking.png', dpi=200)
copy2(FIGURES / 'report_dc_motor_tracking.png', REPORT_IMAGES / 'report_dc_motor_tracking.png')
plt.close(fig)
