# Control III AERO 2 laboratory

Part II of the Automatic Control III laboratory. Develop and validate the controller one question at a time, preserving the supplied course files.

## Start with the simple question models

| Question | Model to study | Basic scopes |
|---|---|---|
| 1 — PI motor-speed control | `simulink/aero2_motor_pi.slx` | Speed, voltage |
| 2 — IP with anti-windup | `simulink/aero2_motor_ip.slx` | Speed, voltage |
| 3 — Mechanical feedback linearization | `simulink/aero2_mechanical_linearization.slx` | Yaw/pitch angles |

The question models contain essential control/plant connections, a basic reference and basic scopes. The IP answer shows `v_raw = I - Kp*omega` and its anti-windup feedback directly. It has no recovery-step generator, PI/IP selector or diagnostic logging bundle.

From the repository as MATLAB's Current Folder:

```matlab
project = setup_project;
open_system(fullfile(project.simulink,'aero2_motor_ip.slx'));
open_system('aero2_motor_ip/Speed scope');
open_system('aero2_motor_ip/Voltage scope');
```

Press Run. The motor examples command main +100 and tail -100 rad/s at 0.1 s. Question 3 starts at zero state and applies a vector acceleration Step `[0.15;-0.1]` rad/s² at 0.2 s, assuming ideal rotor-speed tracking. It does not yet regulate position; the 3 s position controller is Question 4.

The [report](docs/report.md) shows the simple diagrams first, then derives each controller and explains the separate experiment settings and results.

## Separate experiment copies

The diagnostic models are under `simulink/experiments/` and have unique `_experiment` names:

- `aero2_motor_pi_experiment.slx`: signed motor tests and detailed logging.
- `aero2_motor_ip_experiment.slx`: PI/IP selection, optional recovery steps, raw/applied voltage and integral-state diagnostics.
- `aero2_mechanical_linearization_experiment.slx`: acceleration pulses, nonzero initial state, analytical comparison and residual diagnostics.

Open them by full path; `setup_project` returns `project.experiments` without adding that directory to the MATLAB path. For example:

```matlab
open_system(fullfile(project.experiments,'aero2_motor_ip_experiment.slx'));
```

The existing validation commands target these experiment copies:

```matlab
validate_motor_pi;
validate_motor_ip;
validate_feedback_linearization;
validate_question_models;    % Check simple/experiment equivalence; export simple diagrams
export_motor_pi_figures;
export_motor_ip_figures;
export_feedback_linearization_figures;
```

Generated raw recordings and numerical tables go under ignored `results/`. Selected evidence is committed under `docs/figures/`. The simple models have no To Workspace logging blocks; the equivalence check temporarily instruments them in memory and discards those changes.

## Rebuild models

```matlab
project = setup_project;
build_motor_pi;
build_motor_ip;
build_feedback_linearization;
```

Each command recreates its experiment copy and then the simple question answer. The separate `build_*_experiment` functions can rebuild only the diagnostic model. Lasting diagram edits belong in the builders, because rebuilding overwrites generated models.

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

Fetch and recheck target freshness before opening or merging a PR. Rebase only short-lived branches when needed and publish rebases with `--force-with-lease`. Never rebase or force-push `master`. The current private GitHub plan does not enforce server-side branch protection; the existing local pre-push hook is checkout-specific and is not copied by cloning.
