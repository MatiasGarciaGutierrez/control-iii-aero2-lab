"""Export concise Part II plots from existing MATLAB recordings and copy diagrams.

Run from the repository root with:
uv run --with matplotlib --with scipy python validation/export_part2_report_figures.py
"""
from pathlib import Path
from shutil import copy2

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.io import loadmat

ROOT = Path(__file__).resolve().parents[1]
FIGURES = ROOT / 'docs' / 'figures'
IMAGES = ROOT / 'report' / 'images'
plt.rcParams.update({'font.size': 10, 'axes.spines.top': False,
                     'axes.spines.right': False})


def panels():
    fig, axes = plt.subplots(1, 2, figsize=(9, 2.15), layout='constrained')
    for axis in axes:
        axis.grid(alpha=0.2)
        axis.set_xlabel('Time [s]')
    return fig, axes


def save(fig, name):
    fig.savefig(FIGURES / name, dpi=200)
    copy2(FIGURES / name, IMAGES / name)
    plt.close(fig)


def main():
    motor = loadmat(ROOT / 'results/motor_ip_validation.mat', simplify_cells=True)
    fig, axes = panels()
    for index, label in enumerate(['PI', 'IP']):
        run = motor['runs'][index, 0]
        axes[0].plot(run.time, run.data[:, 2], label=label, linewidth=1.6)
        axes[1].plot(run.time, run.data[:, 8], label=label, linewidth=1.6)
    axes[0].plot(run.time, run.data[:, 0], ':k', label='Reference', linewidth=1)
    axes[0].set_ylabel('Speed [rad/s]')
    axes[1].set_ylabel('Voltage [V]')
    for axis in axes:
        axis.set_xlim(0, 0.8)
        axis.legend(frameon=False, fontsize=8)
    save(fig, 'report_part2_motor_comparison.png')

    cascade = loadmat(ROOT / 'results/cascade_control_validation.mat', simplify_cells=True)
    run = cascade['runs'][4]
    ideal = cascade['idealRuns'][4]
    fig, axes = panels()
    for index, axis in enumerate(axes):
        axis.plot(run['time'], run['data'][:, index], ':k', label='Reference')
        axis.plot(run['time'], ideal[:, index + 2], label='Ideal motors', linewidth=1.6)
        axis.plot(run['time'], run['data'][:, index + 18], '--',
                  label='Full cascade', linewidth=1.6)
        axis.set_ylabel(['Yaw [rad]', 'Pitch [rad]'][index])
        axis.set_xlim(0, 5)
        axis.legend(frameon=False, fontsize=8)
    save(fig, 'report_part2_cascade_comparison.png')

    for name in [*(f'question_{index}_diagram.png' for index in range(1, 6)),
                 'position_integral_controller.png']:
        copy2(FIGURES / name, IMAGES / name)


if __name__ == '__main__':
    main()
