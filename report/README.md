# Laboratory report

Open this folder in Codex's LaTeX editor, or edit `main.tex`. Separate Part I and Part II headings distinguish the theoretical exercises from the Quanser AERO 2 application. The report contains concise answers to Part I, Questions 1–3 and optional Question 4, and Part II, Questions 1–5, with optional position integral action, an Experimental validation section on the model run on the AERO 2 and its comparison with simulation, and a Conclusions section.

The compiled [PDF report](lab2_report.pdf) is committed alongside the LaTeX sources for viewing directly on GitHub. Regenerate it whenever the report sources or figures change.

The canonical `lab2_report.pdf` combines Lars’s optional exercise and laboratory account with the updated matched-filter simulations and landscape appendices. The temporary review PDF has been removed from this integration branch; Lars’s branch remains intact.

Models, output derivatives, linearizing laws, coordinate inverses, zero dynamics and gain matching are derived symbolically. Numerical plant parameters and selected gains are introduced only for implementation and simulation. CSTR rates are defined in seconds before deriving the controller; the numerical conversion from the guide's minute-based parameters is stated with the simulation settings.

Numerical values in the report text and tables are rounded to at most four decimal places. Computations and recorded simulation data retain their original precision.

The report is split by question, with short results paragraphs. The title lists Matías García and Lars Snoeijink. Its academic text contains equations, methods and control-performance findings; software test counts and export compatibility details belong in repository documentation. Appendices A and B contain actual Simulink diagram exports for Parts I and II. Equations and figures use part-specific numbering. Selected evidence remains in `../docs/figures/`; matching copies in `images/` make this LaTeX project self-contained for the live preview, which cannot read outside its project folder. Raw MATLAB recordings remain in ignored `../results/`. Diagrams are model exports and plots are logged signals, not UI screenshots.

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
- Question 4 (optional, ball and beam): `simulink/part1_ballbeam.slx` is the simple answer, with a separate `simulink/experiments/part1_ballbeam_experiment.slx` diagnostic copy. These selectively import Lars’s `task1/ex4_ballbeam.slx` handoff; `import_ballbeam_model(sourceFile)` rebuilds them without modifying its source. Parameters are embedded in the model workspace. `validate_ballbeam` reproduces the full nonlinear response, compares independent ODE integration and checks simple/experiment and R2021a-export agreement in R2024b.
- Experimental validation (Part II): `lab_hw_vs_sim.png` is retained as the coauthor’s illustrative reconstruction, combining his R2025b scratch simulation with a device trace extracted from `real_lab_images/image (1).png`. Neither digitized samples nor the scratch simulation model were supplied in this branch; uncertainty is not independently calibrated. Exact trace-derived timing, offsets and causal identification have been removed. The original Scope screenshots remain the physical evidence; `lab_*` diagrams come from the supplied laboratory model. A separate nominal motor calculation in `validate_ballbeam` includes the 180 rad/s tachometer filter; it is not a device measurement.
- All other reported simulations used R2024b. R2021a exports were checked in R2024b, not executed in R2021a. No hardware results are claimed for these simulations.

To regenerate the compact report plots after the respective MATLAB validation scripts have produced `pendulum_regulation_validation.mat` and `dc_motor_tracking_validation.mat`:

```sh
uv run --with matplotlib --with scipy python validation/export_latex_report_figures.py
```

Run that command from the repository root. It reads logged samples from the two MAT recordings and updates both the selected evidence and the report's local image copies; it does not substitute analytical curves for simulation data.

For the CSTR plot, run `setup_project; build_cstr; validate_cstr; export_cstr_figures` in MATLAB. The exporter synchronizes the selected evidence and the local LaTeX image.

To refresh Appendix A, run `setup_project; export_report_block_diagrams`. It exports the saved pendulum, motor and CSTR closed loops and cancellation subsystems, synchronizing `docs/figures/` and `report/images/` without modifying the models or rerunning simulations.

Appendix A also opens each nonlinear plant so the state integrators and balance equations requested in part (a) are visible. The experimental section separates offline filter preparation from the available physical results. It includes two original Scope screenshots with user-confirmed manual disturbances, a qualitative recovery discussion and an evidence-status table. Integral action (`Ki = 1`) was active in both screenshots. Separate speed-loop, individual-axis and integral-comparison records were not supplied. Exact experimental RMS, settling, peak errors and voltage behavior are not inferred from images. A short conclusion and assignment/lecture references follow.

Run `validate_filter_remedies` to reproduce the controlled original/30 rad/s/retuned-gain step comparison and write `results/filter_remedies/steps.csv`, which the Python report exporter uses for its remedy table. The hardware/offline candidate uses `hardware_cascade_parameters` (`Kq=Dq=2.25` numerically, with their respective units) and preserves the 10 rad/s, damping 0.8 filter. The original 5.22 s result remains evidence for the preceding gains; it is not the selected candidate's timing result. This is an offline candidate for professor validation, not tested hardware performance. It was not used on the device; the device runs used `../real_hardware/lin_feedback_Lars_Matias.slx` with the course gains.

The report introduces the supplied yaw/pitch trajectories in Part II, Question 5, where the complete cascade is evaluated. Each earlier question describes its own inputs only where needed. Separate step tests assess settling time. Detailed reference-generation and validation settings remain in repository documentation.

Run `setup_project; validate_course_reference; export_course_reference_figures` in MATLAB first. The baseline validation compares the generated reference against every sample from the original course initializer, checks the saved simple/experiment sources and 60 s duration, compares simple and diagnostic model outputs, and records ideal/noisy tracking and voltage metrics. Temporary logging is discarded without saving model changes; the actual QUARC model is never simulated.

The report now uses the professor-format position/cascade models, whose Derivative blocks act only on the linearly interpolated smooth reference signals. The earlier vectorized models retain analytical `ReferenceRate` feedforward for independent matched checks. In current report experiments, `Kff` switches reference-velocity feedforward while retaining measured-rate feedback. Sharp-step diagnostics replace the reference differentiators by zero sources in memory. The weighted-integral comparison uses the same 180 rad/s tachometer filters, plant, sensor settings and reference implementation as the PD cascade; both outer integrators pause during saturation.

To regenerate the compact motor diagnostic and course-baseline plots, the report's tracking-results table and six diagram copies, run from the repository root:

```sh
uv run --with matplotlib --with scipy python validation/export_part2_report_figures.py
```

The exporter reads `motor_ip_validation.mat`, `position_control_validation.mat` and the fresh `professor_report/comparisons.mat` and `professor_report/metrics.csv` recordings. Question 4's two-panel plot uses the recorded simultaneous 0.2 rad yaw / 0.1 rad pitch steps with ideal motors and velocity feedforward disabled. Time is measured from the command; shading marks the 2% settling bands and vertical lines mark the 3 s target. The Question 5 figure and table use true plant angles from the matched professor-format runs. Older `course_reference_validation.mat` and cascade plots remain historical evidence, not the current table's source.

Part II, Question 1 includes a PI-only speed/voltage response; Question 2 retains the overlaid PI/IP comparison and numerical results table. Both figures use the same recorded main-motor 100 rad/s step at 0.1 s, gains, ideal measurements and time scale. `export_part2_report_figures.py` generates both from `motor_ip_validation.mat`; no analytical replacement traces are used.

The original lab screenshots are preserved in `../real_lab_images/` with provenance notes. Selected documentation/report copies are byte-identical to those originals. The device runs used `../real_hardware/lin_feedback_Lars_Matias.slx` with the unchanged gains of `../real_hardware/nlc_setup_solucion.m`. The lab-ready model gives the nominal controller configuration of this repository's port, which was not used on the device. Screenshots alone do not verify the exact loaded gains, build revision or MATLAB/QUARC release.

Appendix B uses the professor-format diagrams for Part II, Questions 3–5, and includes the opened IP motor controller to show saturated-voltage feedback. Questions 3–4 use ideal-speed mechanics; Question 5 adds the voltage-limited motor loops. Run `export_professor_mechanics_figures` and `export_professor_cascade_figures` after their validation scripts to refresh the canonical PNGs. `export_part2_report_figures.py` copies those diagrams into this self-contained report folder. The numerical results in the main text retain their stated simulation configurations; replacing a layout does not relabel the recorded results.

Questions 3–5 now have individual landscape appendix pages with enlarged plant/controller details. The optional integral loop has its own portrait detail page. Generate the new details and check the integral export with `export_professor_report_details`; this runs its R2021a export in R2024b, without claiming actual R2021a execution.

To reproduce the current Question 5 table and plots: run `setup_project; build_professor_integral; validate_professor_report; export_professor_report_details` in MATLAB, then run `validation/export_part2_report_figures.py` from the repository root. The validator records all three 60 s cases with the supplied sensor effects, signed single-axis/simultaneous step diagnostics with ideal sensors, simple/experiment equivalence, and a saturated optional-integrator freeze test. It also checks the weighted-integral law against the old model under matched filter/reference conventions.

Selected ball-and-beam handoff source SHA-256: `14bde50fdf1d5338488e99b72a4b4197993d40930d16f32401ec786a1c2424ae`. The original handoff file remains unchanged. Raw reproduction data are under ignored `results/ballbeam/`. Its independent full-plant ODE check, simple/experiment comparison and R2021a-export check passed in R2024b; actual R2021a execution remains unverified.
