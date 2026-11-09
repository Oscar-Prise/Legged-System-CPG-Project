# sim/simulink — Simulink version with animation

Same equations as the ode45 code in `sim/`, built like the course models (HW2 `m07.slx`):
fixed-step `ode4`, `dtsim = 1e-4 s`, parameter struct `mp` used by MATLAB Function blocks,
Level-2 MATLAB S-function animation (`sp_anim.m`, same structure as `m07_anim.m`).

| file | what it is |
|---|---|
| `run_cpg_pair.m` | one click: pick a preset, build the model if needed, run, plot |
| `cpg_pair_params.m` | parameters `mp`, initial conditions `ic`, `tmax`, `dtsim`, `rtf_anim`; presets |
| `build_cpg_pair_model.m` | builds `cpg_pair.slx` with add_block / add_line (rebuild any time) |
| `sp_logic_block.m` | code of the MATLAB Function block **Gait Logic and CPG** (discrete, once per step) |
| `sp_eom_block.m` | code of the MATLAB Function block **Leg and Arm Forces** (continuous) |
| `sp_anim.m` | Level-2 S-function: side-view animation (`mp.movie = 1` → mp4, `mp.gif = 1` → cpg_pair.gif) |
| `plot_cpg_pair_results.m` | plots and step/lag/energy summary from `out` after a run |
| `sp_fixedstep_check.m` | runs the two block codes without Simulink (same fixed-step scheme), for checking |

Presets: `stage1_pair` (passive Lanini pair), `stage2_single` (one CPG walker), `stage3_pair` (two CPG walkers).

## Model

    Integrator pos  <-  Integrator vel  <-  [Leg and Arm Forces]  <-  legs
          |                                         ^
          +--> ZOH --> [Gait Logic and CPG] --------+--> Stop if fallen
                                                    +--> Animation, scopes, To Workspace

The gait logic holds contact flags, foot positions and CPG phases in persistent variables,
updated once per step: touchdown when z falls through l0 sin(a0), take-off of the trailing
leg when its force reaches 0, phase reset at touchdown. Forces and accelerations are
continuous and integrated with ode4. The ode45 reference code locates events exactly instead.

## Checks done (Octave 8.4, with `sp_fixedstep_check`, no Simulink)

| case | fixed-step block code | ode45 reference (`sim/`) |
|---|---|---|
| stage2_single, 10 s | step 0.460 s, 1.184 m/s | step 0.458 s, 1.179 m/s |
| stage1_pair, 10 s | step 0.445 s, arm loss 0.706 J | step 0.446 s, arm loss 0.705 J |
| stage3_pair, 20 s, ω2/ω1 = 1 and 1.05 | walks; lag → 0.4° in both | (not in reference) |

Not tested: the Simulink build itself (`build_cpg_pair_model.m`) and the animation inside Simulink —
no MATLAB here. The animation drawing code was checked in Octave with a stand-in block object.
