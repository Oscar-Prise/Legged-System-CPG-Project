# sim/ — paired SLIP walkers (Stage 1) and one CPG walker (Stage 2)

Plain MATLAB (`ode45` + event functions, no toolboxes). Run the scripts from this folder.

| file | what it does |
|---|---|
| `run_stage1a_single_walker.m` | one passive SLIP walker (Geyer 2006): apex-map fixed point, eigenvalues, per-leg GRF |
| `run_stage1b_paired_walkers.m` | Lanini 2017 pair: two walkers + object + spring-damper arms; arm loss per step, phase lag |
| `run_stage2_cpg_walker.m` | one walker + leg damper + CPG-timed push-off: limit cycle (H1), eigenvalues, kick recovery |
| `slip_params.m` | parameter sets `'proposal'` (Geyer Fig. 3A + Lanini Fig. 7 arms) and `'lanini'` (Lanini Table 6) |
| `slip_sim.m` | hybrid simulator (1 or 2 walkers): single/double support, TD/TO/apex/failure events |
| `slip_rhs.m`, `slip_events.m`, `slip_arm.m` | equations of motion, event functions, arm forces |
| `slip_walker_at_apex.m` | initial state at an apex from (dx, y, E) |
| `slip_apex_map.m`, `slip_find_fixed_point.m` | return map R:(dx,y)_i → (dx,y)_{i+1} and Newton fixed-point search |
| `slip_leg.m`, `slip_pushoff.m` | leg force (spring + damper + push-off rest length), push-off profile u(θ) |
| `slip_energy.m`, `slip_idx.m` | total mechanical energy; state-vector layout |
| `slip_post.m`, `slip_gait_metrics.m`, `slip_animate.m` | GRFs, energies, energy balance, step times, phase lag, animation |

## Equations

Walker i, stance leg j with foot at f_j (Geyer 2006 App. A; Lanini 2017 Eqs. 2–5):

    l_j = sqrt((x - f_j)^2 + z^2),     F_j = k (l0 / l_j - 1) [x - f_j ; z]
    m xdd = sum_j F_j,x + F_arm,i       m zdd = sum_j F_j,z - m g

Touchdown when z falls to l0 sin(a0), foot placed at x + l0 cos(a0); take-off when the trailing leg reaches l0.

Arms (horizontal only, Lanini Eq. 6; walker 1 behind, walker 2 in front):

    L1 = xo - x1,  L2 = x2 - xo,   F_a = ka (L_a - L0_a) + ba dL_a/dt
    walker 1: +F1    object: mt xo_dd = -F1 + F2    walker 2: -F2
    dissipated power: ba (dL1^2 + dL2^2)

Stage 2 (switched on with `P.cpg = true`; all off in `slip_params`, so Stage 1 stays passive):

    F_j = k (λ_j - l_j) - c dl_j/dt,        λ_j = l0 + dl · u(θ_j)        (leg damper + push-off)
    dφ/dt = ω - σ Σ_j N_j cos θ_j,           θ_j = φ + (j-1) π            (Owaki & Ishiguro 2017 Eq. 1, N in body weights)
    touchdown of leg j:  θ_j ← θ_TD = 2π(1-β)                              (Aoi & Tsuchiya 2007 Eq. 6)
    a leg leaves the ground when F_j = 0 (the ground only pushes)

u(θ) ramps 0→1 (sin²) starting just after contralateral touchdown (θ = θ_TD + π + 0.1) and holds until the leg is in swing.
Energy book-keeping: E(t) + arm loss + leg loss − actuator work = E(0), actuator power Σ F_spring,j · dλ_j/dt.

## Checks done (Octave 8.4)

- Single walker conserves energy to ~1e-6 J.
- Pair: E_mech(t) + E_diss(t) = E_mech(0) to ~1e-5 J over 30 s.
- `'proposal'` walker: stable fixed point at dx* = 0, y* = 0.971 m, 1.16 m/s, |λ| = 0.978 (Geyer 2006 reports about 1.2 m/s at 816 J).
- Stage 2 (c = 100 Ns/m, dl = 1 cm, ω = 2π/1.0 s, σ = 0): stable limit cycle, 1.18 m/s, |λ| = [0, 0.78, 0.93, 0.93];
  7.1 J push-off work = 7.1 J leg loss per step; recovers from a +10 % speed kick. σ = 3 gives nearly the same cycle.

## Open points

- With a phase reset at every touchdown, the CPG phase is re-set by the mechanics each step, so ω and σ only shift
  the push-off timing slightly (σ = 0 and σ = 3 give almost the same gait). For Stage 3 (frequency mismatch, H2)
  this means the mismatch may barely matter. Options: no reset (pure Owaki feedback) or a partial reset.
- Keep ω·T_step < π + 0.1 and th_hold < π − 0.1, or the rest length jumps under load / legs land extended
  (`slip_sim` stops with an error in the second case).

- Lanini Table 5 gives the start apex height as "2.5 cm" relative to the CoM height with the leg spring at rest. We read this as 2.5 cm *below* l0, because an apex above l0 would mean flight. Not verified.
- Lanini's energy convention is not stated explicitly. We use Geyer's: E = ½ m v² + m g y + ½ k (l0 − l)².
