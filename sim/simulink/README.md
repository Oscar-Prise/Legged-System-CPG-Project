# sim/simulink — Simulink version with animation

Same equations as the ode45 code in `sim/`, built like the course models (HW2 `m07.slx`):
fixed-step `ode4`, `dtsim = 1e-4 s`, parameter struct `mp` used by MATLAB Function blocks,
Level-2 MATLAB S-function animation (`sp_anim.m`, same structure as `m07_anim.m`).

| file | what it is |
|---|---|
| `run_cpg_pair.m` | one click: pick a preset and leg model, rebuild the model if the block code changed, run, plot |
| `cpg_pair_params.m` | parameters `mp`, initial conditions `ic`, `tmax`, `dtsim`, `rtf_anim`; presets; leg-model switch |
| `build_cpg_pair_model.m` | builds `cpg_pair.slx` with add_block / add_line (rebuild any time) |
| `sp_logic_block.m` | code of the MATLAB Function block **Gait Logic and CPG** (discrete, once per step) |
| `sp_eom_block.m` | code of the MATLAB Function block **Leg and Arm Forces** (continuous) |
| `sp_anim.m` | Level-2 S-function: side view + footfall diagram (`mp.movie = 1` → mp4, `mp.gif = 1` → cpg_pair.gif) |
| `plot_cpg_pair_results.m` | plots and step/lag/energy summary from `out` after a run |
| `sp_fixedstep_check.m` | runs the two block codes without Simulink (same fixed-step scheme), for checking |
| `sp_start_state.m` | start state for a chosen initial lag (each walker walks alone first) |

Presets: `stage1_pair` (passive Lanini pair), `stage2_single` (one CPG walker), `stage3_pair` (two CPG walkers).

Leg-model switch (Stage 2 and 3; Stage 1 is always Lanini's passive model):

    preset = 'stage3_pair';  legmodel = 'footmass';  cpg_pair_params     % then Run
    preset = 'stage3_pair';  legmodel = 'damper';    cpg_pair_params

| | `damper` (`mp.legmodel = 1`) | `footmass` (`mp.legmodel = 2`) |
|---|---|---|
| energy sink | leg damper in stance, F = k(λ − l) − c·dl/dt, c = 100 N·s/m | foot of mass m_f = 1 kg per leg stops at touchdown: ΔE = ½ m_f \|v_foot\|² |
| swing | massless leg held at a0 | CPG drives the foot (hip servo, reaction on the body), lands at a0 |
| touchdown | z falls through l0·sin(a0) (Geyer 2006) | foot reaches the ground |
| push-off dl, CPG ω | 0.01 m, 2π/1.0 s | 0.003 m, 2π/0.7 s (swing must finish before the foot lands) |
| drawing | spring \|\| damper leg | spring leg, dot on the swing foot |

Foot target in swing (s = swing progress from the CPG phase, 0 at take-off, 1 at θ_TD; b = 3s² − 2s³):

    r*(s) = (1 − b) r_TO + b (r_hip + l0 [cos a0, −sin a0]) + h_sw sin²(πs) e_z
    F_hip = Ksw (r* − r_f) + Dsw (v* − v_f) + m_f g e_z,   m_f a_f = F_hip − m_f g e_z,   body gets −F_hip

m_f, Ksw, Dsw, h_sw, dl and ω are placeholders chosen so the walker walks; none is fitted to data.

## Pair type and initial lag

    preset = 'stage3_pair'; legmodel = 'damper'; pairtype = 'different'; lag0 = 90; cpg_pair_params

`pairtype = 'identical'`: both 80 kg, l0 = 1 m. `pairtype = 'different'`: body mass and leg length of
Lanini 2017 Pair 05 (Table 1, 2): walker 1 (behind) 57.5 kg, 0.88 m; walker 2 (front) 80 kg, 0.94 m.
All other walker parameters are scaled from the 80 kg / 1 m walker by dynamic similarity
(k ~ m g/l0, c ~ m sqrt(g/l0), ω ~ sqrt(g/l0), E ~ m g l0, push-off and foot lift ~ l0, foot mass ~ m),
so each walker alone walks the same gait in its own time scale. Lanini searched k and a0 per pendulum
instead (Sec. 4.1 step 3); their per-subject values are not given in the paper.

`lag0` (deg): each walker first walks alone for 2 strides (`sp_start_state.m`), walker 2 a further
(360 − lag0)/360 of a stride; the pair then starts from those states with arms relaxed and equal
energies per walker (dE = 0). `lag0 = []` keeps the old start (both at an apex on leg 1, ±5 J).
In 2-D a 180° stride lag is the same body motion as 0° (Lanini Sec. 5.3), so the largest physical
offset is 90° (half a step).

### First results (Octave fixed-step check, 20 s, CPG = leg-damper model, σ = 0, touchdown reset)

| case | start 0° | start 90° |
|---|---|---|
| passive, identical | stays at 0°, walks 20 s | holds ~90° but the arms drain energy (30 J); falls at 11.4 s |
| CPG, identical | stays at 0°, walks 20 s | holds ~87° for 20 s, arm loss 63 J paid by push-off |
| passive, different | no lock, lag slides 8° → 271°; falls at 11.6 s | no lock; falls at 7.8 s |
| CPG, different | no lock, lag slides through all values (≈ 1 stride slip per 18 s); walks 20 s | no lock; walks 20 s |

Alone, the different walkers' strides are 0.834 s and 0.862 s (3.3 % apart, = sqrt(0.88/0.94)).
Whether the CPG pair's 87° is attracting or only neutral (nothing pulls it back) is not tested.

## Animation

Top: side view drawn like Lanini 2017 Fig 6: walker 1 green body, walker 2 red body, object = black
point mass, arms = spring || damper. Leg colours: walker 1 right (leg 1) green, left (leg 2) blue;
walker 2 right red, left purple; gold glow = CPG push-off. Legs: spring (passive), spring || damper (damper model), spring + foot dot in
swing (foot-mass model). The massless swing leg of the passive and damper models is not drawn (as in Fig 6).
The clock above each walker is its CPG phase. The object is drawn at the mean
CoM height (the model's arms are horizontal only).
Bottom: footfall diagram (bar = foot on the ground) for the last 4 s, and the stride phase lag of
walker 2 vs walker 1 (leg-1 touchdowns) with the nearest of Lanini 2017's references
(Sec. 2.2.5: pace ≈ 0°, trot ≈ 180°, diagonal sequence ≈ 225°; 135° shown as its mirror, convention not verified).

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

Energy check: E_mech + arm loss + damper loss + event losses − push-off work − hip work = const
(`R.Ebal`, also logged as `work_log` in Simulink).

| case | fixed-step block code | ode45 reference (`sim/`) | energy check drift |
|---|---|---|---|
| stage1_pair, 10 s | step 0.445 s, arm loss 0.706 J | step 0.446 s, arm loss 0.705 J | 2e-4 J |
| stage2_single damper, 10 s | step 0.460 s, 1.184 m/s | step 0.458 s, 1.179 m/s | 0.06 J |
| stage3_pair damper, 10 s | step 0.460 s, both walkers in step | (not in reference) | 0.11 J |
| stage2_single footmass, 40 s | walks; step 0.430 s, 1.205 m/s, still drifting slowly (0.437 → 0.430 s over the last 30 s) | (no foot model) | 0.16 J |
| stage3_pair footmass, 25 s | walks; step 0.433 s, both in step | (no foot model) | 0.21 J |

Foot model, per step at the end of the 40 s run: impact loss ≈ 0.97 J, hip work ≈ −0.75 J
(picking the resting foot up costs energy too), push-off ≈ 2.1 J. A converged limit cycle for the
foot model is not shown yet.

Not tested: the new block code and animation inside Simulink (no MATLAB here). The animation was
run in Octave with a stand-in block object (that is how the GIFs in `sim/gifs` were made).
