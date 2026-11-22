# Laboratory report

Open this folder in Codex's LaTeX editor, or edit `main.tex`. Separate Part I and Part II headings distinguish the theoretical exercises from the Quanser AERO 2 application. The report contains concise answers to Part I, Questions 1–3, and Part II, Questions 1–5, with optional position integral action and an explicit account of outstanding experimental work.

The compiled [PDF report](lab2_report.pdf) is committed alongside the LaTeX sources for viewing directly on GitHub. Regenerate it whenever the report sources or figures change.

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

Part II summarizes the implementation and measured simulation results documented in `../docs/report.md` and `../docs/hardware.md`. The complete ideal-sensor cascade settles in about 3.2 s; the filtered hardware-interface surrogate takes about 5.2 s and does not meet the 3 s target. Neither result constitutes physical experimental validation.

To regenerate the two compact Part II plots from `motor_ip_validation.mat` and `cascade_control_validation.mat` and synchronize the six existing Part II diagram exports, run from the repository root:

```sh
uv run --with matplotlib --with scipy python validation/export_part2_report_figures.py
```
