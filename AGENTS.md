# Project instructions

## Scope and pace

- This is a solo MATLAB/Simulink course project. Develop one agreed feature at a time and explain the reasoning and simulation evidence to the user.
- Do not restore or import the earlier completed implementation wholesale. The active project starts with the original course materials.
- Keep every file under `course/` unchanged unless the user explicitly requests a source correction. Create working models under `simulink/` and implementation code under `matlab/` as needed.
- Add directories only when they contain meaningful work; do not scaffold empty feature implementations.

## Branches and pull requests

- The user explicitly chose `master` plus short-lived branches, with no `dev` or staging branch. This overrides the two-long-lived-branch Zippedi GitFlow default.
- Apply the Zippedi developer and merge-request skills when available, adapting their GitLab terminology and options to GitHub.
- Prefer Git commands and GitHub API/CLI tools for repository operations. Avoid graphical browser interaction when these tools can perform the task.
- Branch from the latest `origin/master` using `feature/`, `fix/`, `refactor/` or another appropriate short-lived prefix and hyphenated names.
- Every implementation change goes through a PR into `master`. Do not push directly to `master`.
- Assign PRs to `MatiasGarciaGutierrez`. For this solo project, an independent reviewer is optional; do not invent another reviewer or attempt to request review from the PR author.
- Use the PR template, omit irrelevant sections, and describe only the actual diff and validation performed.
- Before opening a PR, fetch the target and confirm its tip is an ancestor of the source. Rebase only a short-lived branch when needed; publish rebases with `--force-with-lease`.
- Opening a PR does not authorize merging it. Merge only when requested, using squash, then remove the short-lived source branch. Never delete or rewrite `master`.
- The private repository's current GitHub plan does not enforce server-side branch protection. Do not claim protection or mandatory peer approval is enforced. Local Git hooks are checkout-specific.

## MATLAB and validation

- Keep new code and models compatible with the laboratory's MATLAB R2021a. Distinguish exporting a model from actually running it in that release.
- Keep identifiers, comments and documentation in American English.
- `setup_project.m` configures explicit project paths. Avoid recursively adding the repository or the original course directory to the MATLAB path.
- Put reproducible experiment scripts and numerical acceptance checks in `validation/` as features are built. Test the behavior relevant to each change.
- Store generated raw data and exports in ignored `results/`. Commit only selected evidence to `docs/figures/` with its explanation.
- Report simulation results separately from hardware results. Experimental validation requires professor approval and the actual device interface and limits.
