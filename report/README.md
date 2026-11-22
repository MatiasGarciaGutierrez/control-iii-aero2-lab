# Laboratory report

Open this folder in Codex's LaTeX editor, or edit `main.tex`. The draft contains concise answers to Part I, Questions 1(a–d) and 2(a–d). Part II has a marked continuation point; its written report will be added later.

The report is split into `question1.tex`, `question2.tex`, and the short motor-results paragraph. Selected evidence remains in `../docs/figures/`; matching copies in `images/` make this LaTeX project self-contained for the live preview, which cannot read outside its project folder. Raw MATLAB recordings remain in ignored `../results/`. The introductory paragraph states the simulation release and 2% settling convention. Figures are plots of logged data or explanatory schematics, not screenshots.

## Build

From this directory, with a TeX installation including `latexmk`:

```sh
latexmk -pdf -interaction=nonstopmode -halt-on-error -outdir=../results/latex-report main.tex
```

The generated PDF is `../results/latex-report/main.pdf`. Supporting build files stay in that ignored folder. The committed sources and PNGs inside this directory are sufficient to compile; Python and MATLAB are not needed for this step.

## Evidence and regeneration

- Question 1: existing pendulum implementation/validation at commit `02b69a9`, on `feature/part-i-pendulum-model` (PR #17). The nominal recorded 2% settling time is 0.976175 s; the mathematical prediction is 0.972320 s. No models from that branch are imported into this report change.
- Question 2(a–b): motor implementation/validation at commit `3ec139c` (PR #18). Question 2(c–d) adds `build_dc_motor_tracking` and `validate_dc_motor_tracking` in the same PR.
- All reported simulations used R2024b. R2021a exports were checked in R2024b, not executed in R2021a. No hardware results are claimed.

To regenerate the compact report plots after the respective MATLAB validation scripts have produced `pendulum_regulation_validation.mat` and `dc_motor_tracking_validation.mat`:

```sh
uv run --with matplotlib --with scipy python validation/export_latex_report_figures.py
```

Run that command from the repository root. It reads logged samples from the two MAT recordings and updates both the selected evidence and the report's local image copies; it does not substitute analytical curves for simulation data.
