# Laboratory report

Open this folder in Codex's LaTeX editor, or edit `main.tex`. Separate Part I and Part II headings distinguish the theoretical exercises from the Quanser AERO 2 application. The report contains concise answers to Part I, Questions 1–3, and Part II, Questions 1–5, with optional position integral action and an explicit account of outstanding experimental work.

The compiled [PDF report](lab2_report.pdf) is committed alongside the LaTeX sources for viewing directly on GitHub. Regenerate it whenever the report sources or figures change.

Models, output derivatives, linearizing laws, coordinate inverses, zero dynamics and gain matching are derived symbolically. Numerical plant parameters and selected gains are introduced only for implementation and simulation. CSTR rates are defined in seconds before deriving the controller; the numerical conversion from the guide's minute-based parameters is stated with the simulation settings.

The report is split by question, with short results paragraphs. The title lists Matías García and Snoeijink Lars. Its academic text contains equations, methods and control-performance findings; software test counts and export compatibility details belong in repository documentation. Appendices A and B contain actual Simulink diagram exports for Parts I and II. Equations and figures use part-specific numbering. Selected evidence remains in `../docs/figures/`; matching copies in `images/` make this LaTeX project self-contained for the live preview, which cannot read outside its project folder. Raw MATLAB recordings remain in ignored `../results/`. Diagrams are model exports and plots are logged signals, not UI screenshots.

## Build

From this directory, with a TeX installation including `latexmk`:

```sh
latexmk -pdf -interaction=nonstopmode -halt-on-error -outdir=../results/latex-report main.tex
cp ../results/latex-report/main.pdf lab2_report.pdf
```

Compilation writes `../results/latex-report/main.pdf`; the second command updates the published copy, `lab2_report.pdf`. Supporting build files stay in the ignored results folder. The committed sources and PNGs inside this directory are sufficient to compile; Python and MATLAB are not needed for this step.

## Evidence and regeneration

- Question 1: the pendulum implementation and evidence are integrated into `master` at `52010d1` through PR #17. The nominal recorded 2% settling time is 0.976175 s; the mathematical prediction is 0.972320 s. The models and validation scripts are available in this checkout.
- Question 2: `build_dc_motor`, `build_dc_motor_linearization`, `build_dc_motor_tracking`, `validate_dc_motor`, and `validate_dc_motor_tracking` reproduce the plant, cancellation and tracking evidence delivered through PR #18.
- Question 3: `build_cstr`, `validate_cstr` and `export_cstr_figures` reproduce the CSTR plant, input-output linearization, internal dynamics and temperature-step results. Time is in seconds, with an explicit factor of 1/60 on the course's minute-based rates. The reported ideal coolant demand is not a practical feasibility claim.
- All reported simulations used R2024b. R2021a exports were checked in R2024b, not executed in R2021a. No hardware results are claimed.

To regenerate the compact report plots after the respective MATLAB validation scripts have produced `pendulum_regulation_validation.mat` and `dc_motor_tracking_validation.mat`:

```sh
uv run --with matplotlib --with scipy python validation/export_latex_report_figures.py
```

Run that command from the repository root. It reads logged samples from the two MAT recordings and updates both the selected evidence and the report's local image copies; it does not substitute analytical curves for simulation data.

For the CSTR plot, run `setup_project; build_cstr; validate_cstr; export_cstr_figures` in MATLAB. The exporter synchronizes the selected evidence and the local LaTeX image.

To refresh Appendix A, run `setup_project; export_report_block_diagrams`. It exports the saved pendulum, motor and CSTR closed loops and cancellation subsystems, synchronizing `docs/figures/` and `report/images/` without modifying the models or rerunning simulations.

Appendix A also opens each nonlinear plant so the state integrators and balance equations requested in part (a) are visible. The experimental section separates offline filter preparation from the available physical results. It includes two original Scope screenshots with user-confirmed manual disturbances, a qualitative recovery discussion and an evidence-status table. Separate speed-loop, individual-axis and integral-comparison records were not supplied. Exact experimental RMS, settling, peak errors and voltage behavior are not inferred from images. A short conclusion and assignment/lecture references follow.

Run `validate_filter_remedies` to reproduce the controlled original/30 rad/s/retuned-gain step comparison and write `results/filter_remedies/steps.csv`, which the Python report exporter uses for its remedy table. The hardware/offline candidate uses `hardware_cascade_parameters` (`Kq=Dq=2.25` numerically, with their respective units) and preserves the 10 rad/s, damping 0.8 filter. The original 5.22 s result remains evidence for the preceding gains; it is not the selected candidate's timing result. This is an offline candidate for professor validation, not tested hardware performance.

The report introduces the supplied yaw/pitch trajectories in Part II, Question 5, where the complete cascade is evaluated. Each earlier question describes its own inputs only where needed. Separate step tests assess settling time. Detailed reference-generation and validation settings remain in repository documentation.

Run `setup_project; validate_course_reference; export_course_reference_figures` in MATLAB first. The baseline validation compares the generated reference against every sample from the original course initializer, checks the saved simple/experiment sources and 60 s duration, compares simple and diagnostic model outputs, and records ideal/noisy tracking and voltage metrics. Temporary logging is discarded without saving model changes; the actual QUARC model is never simulated.

The saved position models now include analytical reference-velocity feedforward. `ReferenceRate` stores the derivative of the same smooth course trajectory, and `Kff=1` enables its contribution. Matched comparisons set `Kff` to 0 and 1 with all other settings fixed. Step, disturbance and recovery diagnostics set `Kff=0` explicitly because their discontinuous references are not differentiated. The optional integral design retains its position-reference weight and outer anti-windup. Regenerate the working/exported models with the position, cascade, integral and hardware/offline builders before rerunning validation after a controller change; R2021a exports are validated in R2024b, not in the laboratory release.

To regenerate the compact motor diagnostic and course-baseline plots, the report's tracking-results table and six diagram copies, run from the repository root:

```sh
uv run --with matplotlib --with scipy python validation/export_part2_report_figures.py
```

The exporter reads `motor_ip_validation.mat`, `course_reference_validation.mat` and `course_reference_metrics.csv`. The baseline figure uses recorded true plant angles; noisy sensor errors are reported separately in the raw metrics. The older `report_part2_cascade_comparison.png` is retained as supplementary step evidence and is not the report's baseline plot.

Part II, Question 1 includes a PI-only speed/voltage response; Question 2 retains the overlaid PI/IP comparison and numerical results table. Both figures use the same recorded main-motor 100 rad/s step at 0.1 s, gains, ideal measurements and time scale. `export_part2_report_figures.py` generates both from `motor_ip_validation.mat`; no analytical replacement traces are used.

The original lab screenshots are preserved in `../real_lab_images/` with provenance notes. Selected documentation/report copies are byte-identical to those originals. The lab-ready model gives the nominal controller configuration; screenshots alone do not verify the exact loaded gains, build revision or MATLAB/QUARC release.
