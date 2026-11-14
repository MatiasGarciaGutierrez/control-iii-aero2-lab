# Control III AERO 2 laboratory

Part II of the Automatic Control III computer-room laboratory. We develop and validate the AERO 2 motor and position controllers one step at a time, starting with the original course files.

## Folder layout

```text
.
├── README.md
├── AGENTS.md
├── .gitignore
├── setup_project.m
├── course/                 Original course documents, script and model
├── matlab/                 Reproducible working-model builder
├── simulink/               Open-loop working model and R2021a export
├── validation/             Simulator checks and figure export
├── docs/                   Growing report and selected figures
└── .github/
    └── pull_request_template.md
```

Implementation is organized as follows; controller and mechanics directories will be added when their first feature needs them:

| Directory | Purpose |
|---|---|
| `matlab/parameters/` | Model parameters and controller gain calculations |
| `matlab/controllers/` | Motor and position control laws |
| `matlab/mechanics/` | Mechanical equations and feedback linearization |
| `simulink/` | Our working models, distinct from the course original |
| `validation/` | Repeatable experiments, plots and measured acceptance checks |
| `docs/` | The developing report and explanations |
| `docs/figures/` | Selected figures committed as evidence |
| `results/` | Generated raw data, plots and exports, excluded from Git |

Do not copy the entire project into separate folders for each question. Feature branches and pull requests record its development.

## Original course files

`course/` contains the assignment, linearization theory, model documentation, initialization script and original Simulink model. Their contents are preserved byte for byte. The original root commit is tagged `course-baseline`; this layout change only relocates those five files.

The supplied model is an unfinished starting point. Its initialization callback refers to the absent `nlc_setup` script and its controller wiring is incomplete. Our separate open-loop model copies its plant unchanged, provides voltage pulses, scopes and logging, and stores the sensor default in its own model workspace. The original course model remains unchanged.

## MATLAB setup

Select the repository as MATLAB's Current Folder and run:

```matlab
project = setup_project;
open_system(fullfile(project.simulink, 'aero2_simulator.slx'))
```

`setup_project` returns absolute project directory paths and adds only existing implementation directories to MATLAB's path. It can be called again after new directories are added. It does not change the Current Folder, initialize the course model, create output directories, or run a simulation. The course originals are intentionally not added to the MATLAB path; open them using their full paths.

The default experiment applies +1 V to the main motor and -1 V to the tail motor from 0.1 to 0.3 s, then observes the response until 1 s. Double-click the three Scope blocks, then press Run to view voltages, rotor speeds, and yaw/pitch. Read the [development report](docs/report.md) for input settings, signal meanings and recorded evidence. This model has no controller yet.

To rebuild and validate it:

```matlab
model = build_simulator;
checks = validate_simulator;
export_simulator_figures;
```

`build_simulator` recreates the working model, so update the builder before rebuilding if you want to retain manual diagram edits. Raw validation data is generated in ignored `results/`; selected figures are exported to `docs/figures/`.

The laboratory uses MATLAB R2021a. The working model includes an R2021a export, but actual R2021a execution remains unverified. Validation is performed in the available R2024b installation.

## Solo development workflow

`master` is the only long-lived branch and is the default branch. This personal project has no staging environment or `dev` branch.

1. Fetch the latest `origin/master` and create a focused `feature/`, `fix/` or `refactor/` branch. Use hyphens in branch names.
2. Implement one agreed step, explain the reasoning, and run the relevant validation.
3. Open a GitHub PR targeting `master`. Assign it to `MatiasGarciaGutierrez`; as a solo project, an independent reviewer is optional.
4. Inspect the diff and evidence before merging. Opening a PR does not authorize merging it.
5. Squash-merge an accepted PR and delete its short-lived branch. Preserve `master`.

Before opening a PR, fetch the latest target and confirm it is an ancestor of the feature branch. Rebase only the short-lived branch if necessary; use `--force-with-lease` when publishing a rebase. Never rebase or force-push `master`.

GitHub reports that server-side branch protections are not enforced for this private repository under the current plan. A local pre-push hook in the current checkout blocks direct pushes and deletions of `master`; hooks are not copied by cloning, so this is not repository-wide enforcement. Follow the PR workflow on every checkout.

Course-provided material retains its original ownership. No new license is assigned to it by this repository.
