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
| `sp_start_state.m` | start state for a chosen initial lag and stance legs (each walker walks alone first) |
| `sp_flight_start.m` | start state at a flight apex (running presets) |
| `sweep_initial_conditions.m`, `plot_ic_sweep.m` | sweep stance legs × initial lag of two identical walkers; table + figures |
| `sp_gait_class.m`, `sp_gait_label.m` | stride lag, drift, lock test and gait name from a `sp_fixedstep_check` run |

Presets: `stage1_pair` (passive Lanini pair), `stage2_single` (one CPG walker), `stage3_pair` (two CPG walkers,
identical parameters and CPG clock by default), `stage2_run` / `stage3_run` (one / two runners, exploratory).

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

## Initial-condition sweep of two identical walkers (2026-11-17)

`sweep_initial_conditions.m` + `plot_ic_sweep.m` (+ `sp_gait_class.m`, `sp_gait_label.m`): both walkers have
exactly the same parameters and the same CPG clock (`pairtype = 'identical'`, `dE = 0`); only the start differs:

    stance0 = [s1 s2]   leg each walker stands on at t = 0 (1 or 2, can differ)
    lag0                stride lag of walker 2 behind walker 1 at t = 0 [deg]

    sw_legmodel = 'damper'; sw_lags = 0:15:165; sw_stance = {[1 1], [1 2], [2 1], [2 2]}; sweep_initial_conditions
    T = plot_ic_sweep({'../results/ic_sweep_damper_a.mat', '../results/ic_sweep_damper_b.mat'}, '../results/ic_sweep_damper')

Each run: 30 s with `sp_fixedstep_check`; the stride lag (walker 2 vs 1, leg-1 touchdowns) over the last 10 s gives
the mean lag, its drift [deg/s] and a label: *locked* (|drift| < 0.3 deg/s) → pace ~0, trot ~180, diagonal
sequence ~225/135, half-step ~90/270; otherwise *drifting to in-step* or *to half-step*; or *fell*.
In 2-D the body motion depends only on the physical offset  p = min(mod(lag,180), 180 − mod(lag,180)) ∈ [0, 90]
(swapping one walker's leg labels adds 180 deg, swapping the walkers negates the lag), so the four `stance0`
combinations are the same physics with different labels; the sweep confirms this (same drift to 0.02 deg/s).

Results (`sim/results/ic_sweep_*.png / .csv / .mat`):

| model | 48 starts, lag0 0–345 deg | outcome |
|---|---|---|
| walking, leg damper | all walk 30 s, none fall | one attractor: **in-step** (0 = pace or 180 = trot, same motion). Drift ≈ −1.4·sin(2·lag) deg/s, so half a step apart (90/270) is an unstable balance point; 120 s runs from 30–105 deg all end within 8 deg of in-step. Intermediate lags (diagonal sequence 225, etc.) are transients only. |
| walking, foot mass | 12 of 48 fall (start 75–105 deg from in-step, fall at 12–14 s) | same single attractor, reached faster (lag within 3 deg after 30 s from 60 deg); starts near half a step apart make the pair fall instead of drifting |
| running pair, leg damper (`stage3_run`, dl = 4 cm, 20 s) | all run 20 s | the **opposite**: half a step apart (90/270) is the attractor, in-step is unstable (15 deg start → 79 deg after 20 s). Only the exact 0/180 start stays in step. |

So with identical walkers the model has **one** paired walking gait (in-step; called pace or trot depending on
which legs are labelled left/right) and **one** paired running gait (half a step apart, i.e. one body at its
apex while the other is loading). The diagonal-sequence lag (225 deg) is never an attractor for identical
walkers; by the proposal's H2 it would need a CPG mismatch (and then the pair must still lock).

Running (`stage2_run`, `stage3_run`, exploratory): the gait logic got a flight phase (`mp.flight = 1`: stance leg
takes off at zero force, the other leg lands when z falls through l0 sin a0; `sp_flight_start.m` makes the
flight-apex start). Passive check: k = 20 kN/m, a0 = 68 deg, 5 m/s (Seyfarth 2002) runs for 10 s with stride
0.505 s, 42 % flight, energy conserved to 1e-3 J. With the leg damper (c = 100) the CPG push-off must be larger:
dl = 1–2 cm the runner slows and stops within 10 s; dl = 3 cm runs at 4.8 m/s (stride 0.72 s), dl = 4 cm at
5.3 m/s (stride 0.515 s). CPG numbers (ω = 2π/0.7 s, β = 0.3, push-off from mid-stance) are placeholders.

Figures and videos of the sweep (`make_result_cases` re-runs 18 representative starts into `../results/ic_cases.mat`,
then `make_result_figures` and `make_result_videos`; the animation is replayed without Simulink through
`sp_anim_replay.m`, the montage through `sp_sweep_movie.m`):

| file (`sim/results/`) | shows |
|---|---|
| `fig_drift_vs_lag.png` | lag drift rate vs lag with sine fits: walking −1.33 sin(2 lag), running +4.59 sin(2 lag) deg/s |
| `fig_gait_map.png` | outcome of every start, three models |
| `fig_lag_vs_time.png`, `fig_footfalls.png` | lag traces of all runs; footfall diagrams of the settled gaits |
| `video_walk_lag60_to_pace.*`, `video_walk_lag60_to_trot.*` | 60 deg start, first 6 s and t = 114–120 s; same motion, walker 2's legs relabelled |
| `video_walk_foot_lag90_falls.*` | foot-mass pair started half a step apart falls at 12.5 s |
| `video_run_lag30_to_halfstep.*`, `video_run_lag0_in_step.*` | running pair locks half a step apart; in-step start stays (unstable) |
| `video_sweep_walking.*`, `video_sweep_running.*` | six starts side by side with the lag traces and a time cursor |

### Poincaré-map check of the gaits (`sp_pair_poincare.m`, `run_poincare_check.m`, `run_poincare_full.m`)

Full return map of the pair, section = walker 1's apex, state = (z1, dx1, x2−x1, z2, dx2, dz2, xo−x1, dxo,
φ1, φ2, stance-foot offsets [+ swing-foot state for the foot model]), Jacobian by central differences
(dt = 2e-5 s), fixed point refined by Newton.  And the 1-D stroboscopic map of the lag from the sweep
data (`fig_poincare_lag_map.png`): lag_{n+1} − lag_n ≈ −A sin(2 lag), slope 1 ∓ 2A at the fixed points.

| gait (leg damper, σ = 0) | full map max \|λ\| | 1-D lag map slope | verdict |
|---|---|---|---|
| walking in step | 0.962 (lag mode: φ2, dz2) then 0.85, 0.85, 0.63 … and 0 for the reset phases | 0.957 | stable |
| walking half step | 1.05 | 1.043 | unstable |
| running half step | 0.99 (Newton residual 1e-3, noisy) | 0.905 | stable |
| running in step | 1.00 (Newton did not converge; map noise) | 1.095 | unstable (from the 1-D map) |
| walking foot mass, σ = 0, in step | no converged fixed point (residual 1e-2; leading modes 0.996 on the swing-foot velocity) | 0.869 | lag mode stable; the walker itself is not on a settled cycle |
| walking foot mass, σ = 3, in step | no fixed point (residual 1e-1, Newton diverges) | — | not a periodic gait: consistent with the σ = 3 sweep (28/48 fall) |
| leg damper, σ = 3, in step | 0.952 | — | stable, same as σ = 0 |

The foot-mass model (the proposal's full model) therefore cannot be certified by the return map yet: its single
walker is not on a converged limit cycle (README above: stride time still drifting after 40 s) and with σ = 3 it
wanders.  Tuning the swing servo / foot mass / push-off until `stage2_single` with `legmodel = 'footmass'` has a
clean fixed point is the prerequisite (`fig_poincare_eigs_full.png`, `poincare_full.mat`).

The lag mode is the slowest mode of the whole pair (walking: 0.96 per stride against 0.85 for the next
mode), which is why the sweeps need ~100 s to settle.  Event quantisation of the fixed-step integrator
limits the full map to about 1e-3 accuracy, so for running the 1-D map is the better number.

### Which CPG model the sweeps used, and the load feedback

All sweeps above used the touchdown reset and **σ = 0** (no load feedback; the Owaki term was off).
The foot-mass sweep is otherwise the proposal's full model (point feet, CPG-driven swing, CPG push-off);
the leg-damper sweep is the fallback model (massless swing leg at a0, damper sink).  Re-run with
**σ = 3** (`sw_sigma = 3`, files `ic_sweep_*_sig3*`):

| model, σ = 3 | result vs σ = 0 |
|---|---|
| leg damper | identical to σ = 0 within 0.02 deg/s (the reset at every touchdown leaves the feedback nothing to do) |
| foot mass | worse: 28 of 48 starts fall (σ = 0: 12). Only starts within ±30 deg of in-step survive; several reach in-step by 6 s and then fall at 8–18 s. The single foot-mass walker with σ = 3 does not settle either (stride time wanders 0.89–0.91 s). |

### Initial CPG phase sweep (`sweep_cpg_phase.m`, `plot_cpg_phase_sweep.m`, `fig_cpg_phase_sweep.png`)

Both walkers start mechanically in step at the same apex state; only the two CPG clocks start at
different phases φ1(0), φ2(0) ∈ {0, π/4, …, 7π/4} (64 starts, leg damper, 30 s).  Needs the new
switches `reset_set` (1 = reset at touchdown, 0 = free-running CPG) and `phi0_set = [φ1 φ2]`.

| CPG | falls | final pattern | transient |
|---|---|---|---|
| reset at TD, σ = 0 | 0 / 64 | all in step | a phase mismatch only changes the first step's push-off: offset up to 52 deg, gone within 20 s |
| reset at TD, σ = 3 | 0 / 64 | all in step | same, settles slightly faster |
| free-running, σ = 3 (no reset) | 29 / 64 | all survivors in step | falls are a single-walker effect: a clock started at ≈ 0.75 π (push-off fires mid-stance of the first step) makes that walker fall whatever its partner does; survivors show offsets up to 90 deg before locking |

A free-running CPG with σ = 0 cannot walk alone (falls at 12.6 s); σ ≥ 3 entrains the clock to the gait
(phase at touchdown locks to a constant), so the load-feedback rule works as the only coupling, as in Owaki 2017.

Other initial conditions not swept yet (all can be set in `cpg_pair_params`): the energy/speed difference
`dE_set` (Lanini used ±15 J), the object's initial position and velocity (arms pre-stretched: `ic.pos(5)`, `ic.vel(5)`),
the apex height / vertical velocity of each walker (a kick off the limit cycle), the CPG phase itself (only matters
without the touchdown reset), and for the foot model the swing-foot start. None of these changes the leg labels,
so none can turn in-step into trot; they can only change *whether* and *how fast* the pair reaches in-step.

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
