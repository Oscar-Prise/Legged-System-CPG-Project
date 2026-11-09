# R16868 Problem Set 3 - Body Articulation

**Overleaf-ready LaTeX report + reproducible running simulation.**

- `main.tex` - main solved report (Problem 1 + project proposal).
- `project_proposal.tex` - separately compiled 11 pt, <=2-page proposal.
- `sections/` - LaTeX source sections.
- `figures/` - actual PDF figures from a fixed-step Python reference solver (not placeholder images).
- `src/run_assignment3.m` - standalone MATLAB implementation with no Simulink requirement; run from MATLAB with `addpath('src'); run_assignment3` to print pivot fits and regenerate MATLAB figures.
- `src/simulink_wiring.md` - exact block equations and integration guidance for editing the original Simulink starter.
- `src/reference_simulation.py` - reference solver used to produce the report's numerical results, figures, and JSON summaries; run `python src/reference_simulation.py` (NumPy and Matplotlib required).
- `course_starter/` - original supplied `m08.slx`, `m08_params.m`, and `m08_anim.m`, preserved unchanged as reference.

## Upload and compile

Upload the ZIP to Overleaf as a new project and compile `main.tex` using **pdfLaTeX**. To compile the proposal alone, change the Overleaf Main document to `project_proposal.tex`.

## Scope and numerical qualifications

The MATLAB source is a **standalone articulated extension** of the course spring-mass running model. It uses the spring, running speed, angle of attack, and numerical time step from the supplied course `m08_params.m`, but uses an explicitly documented simplified single-leg hybrid touchdown/liftoff rule. **The original `m08.slx` is included unchanged, not silently represented as edited.** The embedded PDF plots/numerical results were generated with `src/reference_simulation.py`. This environment had no MATLAB license, so the `.m` source has been reviewed but cannot be claimed execution-tested in MATLAB.

The optional walking extension provides the correct combined-GRF and CoP derivation, but **does not claim a walking numerical run or a measured running-versus-walking pivot difference.** A verified two-leg walking hybrid model is needed for that numerical comparison.

No external bibliography entries are needed. Original class model attribution retained in `course_starter/m08_params.m` and `course_starter/m08_anim.m`.
