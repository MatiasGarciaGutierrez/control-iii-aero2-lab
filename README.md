# Control III AERO 2 laboratory

Parts I and II of the Automatic Control III laboratory. Develop and validate one question at a time, preserving the supplied course files.

## Part I — Question 1(a)

Open [part1_pendulum.slx](simulink/part1_pendulum.slx) and press Run. This is the nonlinear pendulum plant with zero input, an initial angle of 0.1 rad and zero initial angular velocity. Open **Angle scope** to observe the unforced motion. On MATLAB R2021a, use [part1_pendulum_R2021a.slx](simulink/part1_pendulum_R2021a.slx).

The [report](docs/report.md#part-i--question-1a-nonlinear-inverted-pendulum) derives the state equations and explains the blocks, settings and simulation evidence. To rebuild and validate from the repository as MATLAB's Current Folder:

```matlab
setup_project;
build_pendulum;
validate_pendulum;
export_pendulum_figures;
```

## Part I — Question 1(b)

Open [part1_pendulum_linearization.slx](simulink/part1_pendulum_linearization.slx) and press Run. Open **Acceleration scope** to compare virtual input $v$ with actual angular acceleration. Double-click **Linearizing controller** to see the cancellation law. The angle and velocity feed back from the original nonlinear plant. The input is a 0.2 rad/s² step at 0.2 s; this demonstrates linearization without stabilization. An [R2021a export](simulink/part1_pendulum_linearization_R2021a.slx) is included.

```matlab
setup_project;
build_pendulum_linearization;
validate_pendulum_linearization;
export_pendulum_linearization_figures;
```

## Part I — Question 1(c)

Open [part1_pendulum_regulation.slx](simulink/part1_pendulum_regulation.slx) and press Run. **Linear state feedback** computes $v=-36\theta-12\dot\theta$ ahead of the nonlinear cancellation controller. From the initial state `[0.1; 0]`, **Angle scope** shows critically damped convergence to upright, with a predicted 2% settling time of 0.9723 s. The [R2021a export](simulink/part1_pendulum_regulation_R2021a.slx) is included. Reproduce with `build_pendulum_regulation`, `validate_pendulum_regulation` and `export_pendulum_regulation_figures` after `setup_project`.

## Part II — Start with the simple question models

| Activity | File to open in Simulink | Available views |
|---|---|---|
| Preparation | [aero2_simulator.slx](simulink/aero2_simulator.slx) | Voltage, rotor speed/acceleration, body angle/rate/acceleration |
| 1 — PI motor-speed control | [aero2_motor_pi.slx](simulink/aero2_motor_pi.slx) | Speed, speed error, motor acceleration, voltage |
| 2 — IP with anti-windup | [aero2_motor_ip.slx](simulink/aero2_motor_ip.slx) | Speed, error, motor acceleration, applied/raw voltage, motor integral |
| 3 — Feedback linearization | [aero2_mechanical_linearization.slx](simulink/aero2_mechanical_linearization.slx) | Body angle, rate, commanded/actual acceleration |
| 4 — Ideal position control | [aero2_position_control.slx](simulink/aero2_position_control.slx) | Body angle, rate, commanded/actual acceleration, position error, rotor commands |
| 5 — Complete cascade | [aero2_cascade_control.slx](simulink/aero2_cascade_control.slx) | Body motion, position error, rotor speed/acceleration, voltage and motor integral |
| Optional — Position integral action | [aero2_position_integral.slx](simulink/aero2_position_integral.slx) | Body motion, rotor speed/acceleration, voltage, motor and position integrals |

Open the file in the table, press **Run**, then double-click a Scope. Use the files directly under `simulink/` for the activity answers. Files under `simulink/experiments/` are the separate diagnostic tests. On laboratory MATLAB R2021a, open the matching `_R2021a.slx` export (for example, `aero2_motor_pi_R2021a.slx`).

The question models contain essential control/plant connections, a basic reference and basic scopes. The IP answer shows `v_raw = I - Kp*omega` and its anti-windup feedback directly. It has no recovery-step generator, PI/IP selector or diagnostic logging bundle.

From the repository as MATLAB's Current Folder:

```matlab
project = setup_project;
open_system(fullfile(project.simulink,'aero2_motor_ip.slx'));
open_system('aero2_motor_ip/Speed scope');
open_system('aero2_motor_ip/Voltage scope');
```

Press Run. The motor examples command main +100 and tail -100 rad/s at 0.1 s. Question 3 starts at zero state and applies a vector acceleration Step `[0.15;-0.1]` rad/s² at 0.2 s, assuming ideal rotor-speed tracking. Question 3 does not regulate position. Its `Angle scope` shows yaw/pitch angles, `Rate scope` shows body angular velocities, and `Acceleration scope` compares commanded and actual body angular accelerations. The report's pulse experiment uses different initial conditions and a pulse instead of the simple model's step. Question 4 uses the inverse to track desired yaw/pitch angles, still assuming perfect motor tracking.

**Questions 4–5 and the optional controller now use the course reference by default.** Press Run for the complete 60 s yaw/pitch trajectory. Double-click **Desired orientation** to see the two From Workspace sources: `U_v` (yaw) and `U_h` (pitch). The exact course time series, including the factor of two, are stored in each model workspace; no initialization script is needed before running a saved model. All Scopes display 60 s.

The position experiment copies also default to the course trajectory. Validation scripts explicitly select the separate step, disturbance and recovery tests. Their measured step settling times remain 2.996 s for Question 4 and 3.204–3.210 s for Question 5. Course-trajectory plots show tracking performance, not step settling time.

The optional model adds position integral action and outer anti-windup without replacing Question 5. Nominal settling is 3.042–3.050 s. The separate experiment compares rejection of constant external torques and recovery after temporary voltage-authority loss. See the optional section in the report for the gain derivation and tradeoffs.

The [report](docs/report.md) shows the simple diagrams first, then derives each controller and explains the separate experiment settings and results.

## Separate experiment copies

The diagnostic models are under `simulink/experiments/` and have unique `_experiment` names:

- `aero2_motor_pi_experiment.slx`: signed motor tests and detailed logging.
- `aero2_motor_ip_experiment.slx`: PI/IP selection, optional recovery steps, raw/applied voltage and integral-state diagnostics.
- `aero2_mechanical_linearization_experiment.slx`: acceleration pulses, nonzero initial state, analytical comparison and residual diagnostics.
- `aero2_position_control_experiment.slx`: signed single-axis/simultaneous angle tests and position-loop diagnostics.
- `aero2_cascade_control_experiment.slx`: complete controller, true/measured state logging, sensor tests and the supplied smooth trajectory.
- `aero2_position_integral_experiment.slx`: optional integral action, disturbance injection and outer anti-windup comparisons.

Open them by full path; `setup_project` returns `project.experiments` without adding that directory to the MATLAB path. For example:

```matlab
open_system(fullfile(project.experiments,'aero2_motor_ip_experiment.slx'));
```

The existing validation commands target these experiment copies:

```matlab
validate_motor_pi;
validate_motor_ip;
validate_feedback_linearization;
validate_course_reference;  % Verify the exact course inputs and all three default trajectories
export_course_reference_figures;
validate_position_control;
validate_cascade_control;
validate_activity_scopes;    % Audit Scope connections, actual accelerations and exports
validate_question_models;    % Check simple/experiment equivalence; export simple diagrams
export_motor_pi_figures;
export_motor_ip_figures;
export_feedback_linearization_figures;
export_position_control_figures;
export_cascade_control_figures;
validate_position_integral;
validate_position_integral_models;
export_position_integral_figures;
```

Generated raw recordings and numerical tables go under ignored `results/`. Selected evidence is committed under `docs/figures/`. The simple models have no To Workspace logging blocks; the equivalence check temporarily instruments them in memory and discards those changes.

## Rebuild models

```matlab
project = setup_project;
build_motor_pi;
build_motor_ip;
build_feedback_linearization;
build_position_control;
build_position_control_experiment;
build_cascade_control;
build_cascade_control_experiment;
build_position_integral;
build_position_integral_experiment;
```

The Question 1–3 builders recreate their experiment copy and then the simple answer. For Questions 4–5, build each simple model first and its experiment copy second, as shown above. The separate `build_*_experiment` functions can rebuild only the diagnostic model. Lasting diagram edits belong in the builders, because rebuilding overwrites generated models.

`simulink/aero2_simulator.slx` remains the open-loop preparation model. Use `build_simulator`, `validate_simulator` and `export_simulator_figures` for that baseline.

All working models have R2021a exports. Validation uses the available MATLAB R2024b installation; running an exported model in R2024b is distinct from actual execution in R2021a. Physical testing requires professor approval and the actual device interface and limits.

## Folder layout

```text
course/                  Original supplied PDFs, initialization script and model
matlab/                  Model builders
matlab/parameters/       Course constants and controller gains
matlab/mechanics/        Mechanical equations and nonlinear inverse
simulink/                Simple question answers and open-loop preparation
simulink/experiments/    Separate diagnostic models
validation/              Repeatable experiments and acceptance checks
docs/                    Growing report and selected figures
results/                 Generated data and metrics, ignored by Git
```

`setup_project.m` returns absolute paths and adds only explicit implementation directories. It does not initialize models, change folder or run experiments. Course originals and experiment models are intentionally excluded from the MATLAB path.

The five files under `course/` remain byte-identical to the `course-baseline` tag. The supplied unfinished model remains unchanged, including its original callback; working models are separate. Course-provided material retains its original ownership; this repository assigns no new license to it.

## Solo development workflow

`master` is the only long-lived branch; there is no staging or `dev` branch.

1. Branch from the latest `origin/master` with a focused `feature/`, `fix/` or `refactor/` name.
2. Implement one agreed step, document its reasoning and validate it.
3. Open a GitHub PR targeting `master`, assigned to `MatiasGarciaGutierrez`. Independent review is optional for this solo project.
4. Merge only when requested, using squash, then remove the source branch.

Fetch and recheck target freshness before opening or merging a PR. Rebase only short-lived branches when needed and publish rebases with `--force-with-lease`. Never rebase or force-push `master`. Check current branch protection in GitHub settings; the existing local pre-push hook is checkout-specific and is not copied by cloning.
