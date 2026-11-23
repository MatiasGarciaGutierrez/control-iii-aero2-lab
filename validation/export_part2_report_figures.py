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
    fig, axes = plt.subplots(1, 2, figsize=(9, 1.8), layout='constrained')
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
    pi = motor['runs'][0, 0]
    fig, axes = panels()
    axes[0].plot(pi.time, pi.data[:, 2], label='PI', linewidth=1.6)
    axes[0].plot(pi.time, pi.data[:, 0], ':k', label='Reference', linewidth=1)
    axes[1].plot(pi.time, pi.data[:, 8], label='PI', linewidth=1.6)
    axes[0].set_ylabel('Speed [rad/s]')
    axes[1].set_ylabel('Voltage [V]')
    for axis in axes:
        axis.set_xlim(0, 0.8)
        axis.legend(frameon=False, fontsize=8)
    save(fig, 'report_part2_pi_response.png')
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

    position = loadmat(ROOT / 'results/position_control_validation.mat', simplify_cells=True)
    run = position['runs'][4]  # Recorded simultaneous +0.2/+0.1 rad steps.
    step_time = float(position['p']['stepTime'])
    selected = run['time'] >= step_time
    elapsed = run['time'][selected] - step_time
    fig, axes = panels()
    for index, axis in enumerate(axes):
        target = float(run['data'][-1, index])
        axis.axhspan(0.98*target, 1.02*target, color='0.8', alpha=0.5)
        axis.axvline(3, color='0.4', linestyle=':', linewidth=1)
        axis.plot(elapsed, run['data'][selected, index+2], label='Simulink', linewidth=1.6)
        axis.axhline(target, color='black', linestyle='--', linewidth=0.9, label='Reference')
        axis.set_xlim(0, 5)
        axis.set_xlabel('Time after step [s]')
        axis.set_ylabel(['Yaw [rad]', 'Pitch [rad]'][index])
        axis.legend(frameon=False, fontsize=7)
    save(fig, 'report_part2_position_steps.png')

    baseline = loadmat(ROOT / 'results/course_reference_validation.mat', simplify_cells=True)
    # The empty ideal-mechanics noise cell keeps scipy's noisyRuns entries as
    # MATLAB structs rather than dictionaries; normalize at the file boundary.
    noisy = baseline['noisyRuns'][1]
    run = {'time': noisy.time, 'data': noisy.data}
    before = baseline['comparisonRuns'][1, 1, 0]
    fig, axes = plt.subplots(2, 3, figsize=(9, 2.7), layout='constrained')
    for index in range(2):
        tracking, error, voltage = axes[index]
        tracking.plot(run['time'], run['data'][:, index], ':k', label='Course reference')
        tracking.plot(run['time'], run['data'][:, index + 18], label='With feedforward', linewidth=1.1)
        tracking.set_ylabel(['Yaw [rad]', 'Pitch [rad]'][index])
        for record, label, color in [(before, 'Without', '0.6'), (noisy, 'With', 'C0')]:
            error.plot(record.time, record.data[:, index]-record.data[:, index+18],
                       label=label, color=color, linewidth=0.9)
            voltage.plot(record.time, record.data[:, index+14], label=label, color=color, linewidth=0.65)
        error.set_ylabel('Error [rad]')
        voltage.set_ylabel(['Main [V]', 'Tail [V]'][index])
        for axis in axes[index]:
            axis.set_xlim(0, 60)
            axis.grid(alpha=0.2)
            axis.set_xlabel('Time [s]')
            axis.legend(frameon=False, fontsize=6)
    save(fig, 'report_part2_course_baseline.png')

    with (ROOT / 'results/course_reference_metrics.csv').open() as stream:
        metrics = list(csv.DictReader(stream))
    cases = [('IP cascade, without FF', 'aero2_cascade_control', 0),
             ('IP cascade, with FF', 'aero2_cascade_control', 1),
             ('Integral cascade, with FF', 'aero2_position_integral', 1)]
    rows = []
    for label, model, feedforward in cases:
        pair = [row for row in metrics if row['Model'] == model
                and int(row['MeasurementEffects']) == 1
                and int(row['ReferenceFeedforward']) == feedforward]
        yaw, pitch = sorted(pair, key=lambda row: int(row['Axis']))
        voltage = '---' if model == 'aero2_position_control' else f"{float(yaw['PeakVoltage']):.2f}"
        rows.append(f"{label} & {float(yaw['TrackingRMS']):.4f}/{float(pitch['TrackingRMS']):.4f} & "
                    f"{float(yaw['PeakError']):.4f}/{float(pitch['PeakError']):.4f} & {voltage}" + r'\\')
    table = [r'\begin{center}', r'\begin{tabular}{lccc}', r'\hline',
             r'Controller & RMS yaw/pitch (rad) & Peak error yaw/pitch (rad) & Peak $|v|$ (V)\\\hline',
             *rows, r'\hline', r'\end{tabular}', r'\end{center}']
    (ROOT / 'report/course_baseline_results.tex').write_text('\n'.join(table) + '\n')

    with (ROOT / 'results/filter_remedies/steps.csv').open() as stream:
        remedies = list(csv.DictReader(stream))
    rows = []
    for key, label in [('Original', 'Original gains, 10 rad/s'),
                       ('Filter 30', 'Original gains, 30 rad/s'),
                       ('Gains 2.25', 'Retuned gains, 10 rad/s'),
                       ('Gains 2.25 direct', 'Retuned gains, direct rates')]:
        group = [row for row in remedies if row['Case'] == key]
        times = [float(row['SettlingTime']) for row in group]
        overshoot = max(float(row['OvershootPercent']) for row in group)
        voltage = max(float(row['PeakVoltage']) for row in group)
        settling = f'{min(times):.3f}' if max(times)-min(times)<0.0005 else f'{min(times):.3f}--{max(times):.3f}'
        rows.append(f'{label} & {settling} & {overshoot:.3f} & {voltage:.2f}' + r'\\')
    table = [r'\begin{center}', r'\begin{tabular}{lccc}', r'\hline',
             r'Simulated configuration & $T_s$ (s) & Overshoot (\%) & Peak $|v|$ (V)\\\hline',
             *rows, r'\hline', r'\end{tabular}', r'\end{center}']
    decision = (r'The selected device candidate uses $K_q=2.25I_2$ s$^{-2}$ and '
                r'$D_q=2.25I_2$ s$^{-1}$, retaining the template filter. '
                r'Raising its cut-off is an alternative that may increase noise. '
                r'The table uses signed simultaneous $[0.2,0.1]^\mathsf{T}$ rad steps, '
                r'ideal sensors and no reference differentiation. Nominal simulator gains remain unchanged.')
    (ROOT / 'report/filter_remedy_results.tex').write_text('\n'.join(table) + '\n' + decision + '\n')

    for name in [*(f'question_{index}_diagram.png' for index in range(1, 6)),
                 'position_integral_controller.png',
                 'professor_format_question3.png', 'professor_format_question4.png',
                 'professor_format_cascade.png', 'professor_format_ip.png']:
        copy2(FIGURES / name, IMAGES / name)


if __name__ == '__main__':
    main()
