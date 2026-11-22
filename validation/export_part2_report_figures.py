"""Export concise Part II plots from existing MATLAB recordings and copy diagrams.

Run from the repository root with:
uv run --with matplotlib --with scipy python validation/export_part2_report_figures.py
"""
from pathlib import Path
from shutil import copy2
import csv

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

    baseline = loadmat(ROOT / 'results/course_reference_validation.mat', simplify_cells=True)
    ideal = baseline['runs'][0]
    # The empty ideal-mechanics noise cell keeps scipy's noisyRuns entries as
    # MATLAB structs rather than dictionaries; normalize at the file boundary.
    noisy = baseline['noisyRuns'][1]
    run = {'time': noisy.time, 'data': noisy.data}
    fig, axes = panels()
    for index, axis in enumerate(axes):
        axis.plot(run['time'], run['data'][:, index], ':k', label='Course reference')
        axis.plot(ideal['time'], ideal['data'][:, index + 2], label='Ideal motors', linewidth=1.6)
        axis.plot(run['time'], run['data'][:, index + 18], '--',
                  label='IP cascade, course sensors', linewidth=1.3)
        axis.set_ylabel(['Yaw [rad]', 'Pitch [rad]'][index])
        axis.set_xlim(0, 60)
        axis.legend(frameon=False, fontsize=7)
    save(fig, 'report_part2_course_baseline.png')

    with (ROOT / 'results/course_reference_metrics.csv').open() as stream:
        metrics = list(csv.DictReader(stream))
    cases = [('Ideal motors', 'aero2_position_control', 0),
             ('Full IP cascade', 'aero2_cascade_control', 0),
             ('Full IP cascade', 'aero2_cascade_control', 1),
             ('Integral cascade', 'aero2_position_integral', 1)]
    rows = []
    for label, model, mode in cases:
        pair = [row for row in metrics if row['Model'] == model
                and int(row['MeasurementEffects']) == mode]
        yaw, pitch = sorted(pair, key=lambda row: int(row['Axis']))
        voltage = '---' if model == 'aero2_position_control' else f"{float(yaw['PeakVoltage']):.2f}"
        rows.append(f"{label} & {'Course' if mode else 'Ideal'} & "
                    f"{float(yaw['TrackingRMS']):.5f} & {float(pitch['TrackingRMS']):.5f} & {voltage}" + r'\\')
    table = [r'\begin{center}', r'\begin{tabular}{llccc}', r'\hline',
             r'Controller & Sensors & Yaw RMS (rad) & Pitch RMS (rad) & Peak $|v|$ (V)\\\hline',
             *rows, r'\hline', r'\end{tabular}', r'\end{center}']
    (ROOT / 'report/course_baseline_results.tex').write_text('\n'.join(table) + '\n')

    for name in [*(f'question_{index}_diagram.png' for index in range(1, 6)),
                 'position_integral_controller.png']:
        copy2(FIGURES / name, IMAGES / name)


if __name__ == '__main__':
    main()
