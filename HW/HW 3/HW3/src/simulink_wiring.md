# How to integrate the articulated model into the course `m08.slx`

This is a **block-level wiring guide**, not a claim that the original `.slx` has already been modified or tested in Simulink. The submission's working implementation is `src/run_assignment3.m` and the independently executed numerical reference is `src/reference_simulation.py`.

## 1. Initialize the model

1. Run the original `course_starter/m08_params.m` with `wlk_flag = 0`.
2. Add `mp.J = 3.5; mp.d = 0.2; mp.h = 0.2;` to the MATLAB workspace; keep `mp.m = 70; mp.k = 20000; mp.l0 = 1; mp.aoa = deg2rad(68); dtsim = 1e-4`.
3. Add trunk-angle and trunk-angular-rate integrators to the EoM subsystem: initialize `phi0 = -0.02`, `dphi0 = 0` (or any stated initial perturbation).
4. The original point-mass `ic.z0=1.1` represents the **hip height** in the articulated model, so initialize the new center of mass as `z_C0 = ic.z0 + mp.d*cos(phi0)`.
5. Change the leg attachment point fed to the leg geometry and original foot-placement logic from the point-mass CoM to the new hip point:

   `hip_x = x_C + d*sin(phi)`; `hip_z = z_C - d*cos(phi)`.

## 2. EoM subsystem

Replace point-mass accelerations by

```matlab
xddot   = Fx/mp.m;
zddot   = Fz/mp.m - mp.g;
phiddot = (tau + mp.d*(Fx*cos(phi) + Fz*sin(phi)))/mp.J;
```

Integrate to `x_C`, `z_C`, `phi`, and their rates. For running, feed the GRF and hip torque of the currently loaded leg; during flight all contact terms must be zero.

## 3. Stance leg and spring force

Calculate `xrel=hip_x-foot_x`, `zrel=hip_z-foot_z`, `l=hypot(xrel,zrel)`, and `Fs=max(0,mp.k*(mp.l0-l))`. Export `Fs` as a signal from the leg-force subsystem **before** the torque-dependent GRF calculation. Then calculate

```matlab
Fx = (xrel/l)*Fs + (zrel/l^2)*tau;
Fz = (zrel/l)*Fs - (xrel/l^2)*tau;
```

Guard the divisions for noncontact / degenerate leg length and ensure zero force in swing. The GRF convention is the force on the body (upward positive), matching the homework equations.

## 4. Two alternative controller blocks

The load-modulated PD block takes `phi, dphi, Fs`:

```matlab
Kp = 200; Kd = 30;
tauPD = (Fs/mp.m)*(-Kp*phi - Kd*dphi);
```

The geometric VPP block takes `xrel, zrel, phi, Fs`:

```matlab
ell = hypot(xrel,zrel); L = mp.d + mp.h;
den = ell^2 + L*(zrel*cos(phi) - xrel*sin(phi));
tauVPP = -Fs*ell*L*(xrel*cos(phi)+zrel*sin(phi))/den;
```

Use a Manual Switch / Multiport Switch before the GRF calculation to select `tauPD` or `tauVPP`. The optional pitch damping term is `tauDamp = -10*(Fs/mp.m)*dphi`. Add it *after* the VPP output when comparing damping effects; exact VPP collinearity is then generally lost. Check `abs(den)>eps` and do not silently claim exact collinearity in a singular configuration.

If the original monolithic MATLAB Function block causes Simulink to report an algebraic loop because torque enters the same direct-feedthrough block as the load output, place a **Memory** block (or one-step discrete Unit Delay for fixed-step operation) in the `Fs -> PD` load-feedback path, or split spring-force calculation into its own block so that `Fs` can be evaluated before `tau`. The stand-alone solver's separated calculation does not need a numerical delay.

## 5. Stance events and outputs

Keep the original model's running swing/stance logic where practical, now referencing the hip location. The stand-alone model instead uses a simplified single-leg event rule documented in the report: touchdown when descending hip height crosses `l0*sin(aoa)`, placing the foot at `hip_x+l0*cos(aoa)`, and liftoff when an extending leg exceeds its rest length (with a 30 ms minimum dwell).

Log `t,x_C,z_C,phi,dphi,foot_x,foot_z,Fs,Fx,Fz,tau,stance`. In a MATLAB analysis script transform the foot-to-CoM position and GRF using

```matlab
R = [cos(phi), sin(phi); -sin(phi), cos(phi)];
foot_body = R*([foot_x;foot_z]-[x_C;z_C]);
F_body = R*[Fx;Fz];
```

Then plot the full force **lines of action** from `foot_body`, fit a least-squares intersection, and compare the fit against `(0, h)` in the trunk frame. The fitting algorithm is included in `run_assignment3.m`.

## 6. Optional walking and double support

Do not reuse the single-leg running rule as though it were a verified two-leg walking model. Restore `wlk_flag = 1` and compute both active stance-leg forces and torques before forming

```matlab
Fsum = F_R + F_L;
CoP_x = (footR_x*F_R(2)+footL_x*F_L(2))/(F_R(2)+F_L(2));
```

The CoP formula is valid for both feet on a common flat ground level when the net vertical GRF is nonzero. The optional numerical comparison additionally requires verification of the two-leg stance/swing transitions and the chosen torque distribution.
