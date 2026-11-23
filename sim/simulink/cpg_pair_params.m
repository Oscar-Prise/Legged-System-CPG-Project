%
% cpg_pair_params.m -- Parameters for the Simulink model cpg_pair.slx
%
% Two 2-D spring-mass walkers (Geyer, Seyfarth & Blickhan 2006) holding an object
% through horizontal spring-damper arms (Lanini et al. 2017, Sec. 3.2), each with a
% CPG-timed push-off (Owaki & Ishiguro 2017 Eq. 1, reset at touchdown as in
% Aoi & Tsuchiya 2007).  Same equations as ../slip_*.m (the ode45 reference code).
%
% Choose a preset (and a leg model) BEFORE running this script, e.g.
%     preset = 'stage2_single';  legmodel = 'footmass';  cpg_pair_params
% then press Run in the model.  Defaults: preset 'stage1_pair', legmodel 'damper'.
%
%   'stage1_pair'    passive Lanini pair (no CPG, no leg damper)          Stage 1
%   'stage2_single'  one walker with leg damper + CPG push-off             Stage 2
%   'stage3_pair'    two CPG walkers coupled by the object (first look)    Stage 3
%   'stage2_run'     one RUNNER (flight phase on, k = 20 kN/m, a0 = 68 deg, 5 m/s)   exploratory
%   'stage3_run'     two coupled runners                                              exploratory
%
% Leg model switch (energy sink of the CPG walker; Stage 2 and 3 only):
%   'damper'    leg damper c in stance, massless swing leg held at a0 (our Stage 2 choice)
%   'footmass'  point foot of mass mf, CPG-driven swing, energy lost when the foot lands
%               (the sink in our proposal)
%
% Pair type:
%   'identical'  both walkers 80 kg, l0 = 1 m (Geyer 2006; Lanini 2017 Sec. 4.2)
%   'different'  body mass and leg length of Lanini 2017 Pair 05 (Table 1, 2):
%                walker 1 (behind) = subject 5, 57.5 kg, 0.88 m; walker 2 (front) = subject 3,
%                80 kg, 0.94 m.  Stiffness, damping, push-off, foot mass, CPG frequency and energy
%                are scaled from the 80 kg / 1 m walker by dynamic similarity (k ~ m g/l0,
%                c ~ m sqrt(g/l0), omega ~ sqrt(g/l0), E ~ m g l0), so each walker alone walks
%                the same gait in its own time scale.  Lanini instead searched k and a0 for each
%                pendulum (Sec. 4.1, step 3); their per-subject values are not in the paper.
%
% Initial conditions of the pair (the walkers' parameters stay identical; only the start differs):
%   lag0 = []    both walkers start at an apex on leg 1 at t = 0 (in step)
%   lag0 = 90    each walker first walks alone for 2 strides (sp_start_state.m), walker 2 a further
%                (360 - lag0)/360 of a stride, so the pair starts with a stride lag of about lag0 deg
%   stance0 = [s1 s2]   leg (1 = right, 2 = left) that walker 1 and 2 stand on at t = 0;
%                default [1 1].  [1 2]: walker 2 starts on the other leg (trot-like labels).
%                In 2-D this only relabels the legs (stride lag + 180 deg), the body motion is
%                the same (Lanini 2017 Sec. 5.3).
%   omega_ratio_set   H2: walker 2's CPG clock rate relative to walker 1 (1 = matched, 1.05 = 5 % faster)
%   sigma_set, reset_set, phi0_set, dl_set   load-feedback gain, touchdown reset on/off, initial CPG phases, push-off
%   dE_set       energy difference between the walkers [J] (walker 1: E0 - dE, walker 2: E0 + dE).
%                Default: 5 J for the passive Stage 1 pair (Lanini Table 5), 0 for the CPG pair.
%
% 'stage3_pair' with pairtype 'identical' (the default) makes the two walkers exactly alike:
% same body (m, l0, k, a0), same leg damper / foot, same push-off dl and the same CPG clock
% (omega, sigma, beta, th_TD, th_on, th_ramp, th_hold) and the same start energy.  The two
% walkers then differ only in their initial conditions (lag0, stance0).
%
% Example:  preset = 'stage3_pair'; legmodel = 'damper'; lag0 = 45; stance0 = [1 2];
%           cpg_pair_params
%

if ~exist('preset', 'var') || isempty(preset), preset = 'stage1_pair'; end
if ~exist('legmodel', 'var') || isempty(legmodel), legmodel = 'damper'; end
if ~exist('pairtype', 'var') || isempty(pairtype), pairtype = 'identical'; end
if ~exist('lag0', 'var'), lag0 = []; end
if ~exist('stance0', 'var') || isempty(stance0), stance0 = [1 1]; end
if ~exist('dE_set', 'var'), dE_set = []; end

mp = [];

% Walker (Geyer 2006 Fig. 3A); values for an 80 kg walker with l0 = 1 m, made per walker below
% ----------------------------------------------------------------------------------------------
mp.g  = 9.81;          % [m/s^2]
mp.m  = 80;            % [kg]    walker mass
mp.l0 = 1.0;           % [m]     leg rest length
mp.k  = 14e3;          % [N/m]   leg stiffness
mp.a0 = 69*pi/180;     % [rad]   angle of attack of the swing leg

% Arms and object (Lanini 2017, Fig. 7 caption; object 8 kg, Sec. 2)
% -------------------------------------------------------------------
mp.mt = 8;             % [kg]    object mass
mp.ka = 3.5e3;         % [N/m]   arm stiffness (each arm)
mp.ba = 320;           % [Ns/m]  arm damping   (each arm)

% Stage 2 additions (leg damper + CPG push-off); switched on by the presets below
% --------------------------------------------------------------------------------
mp.c       = 0;                    % [Ns/m]  leg damper
mp.cpg     = 0;                    %         1 = CPG push-off on
mp.dl      = 0;                    % [m]     push-off leg extension
mp.omega   = 2*pi*[1 1];           % [rad/s] CPG frequency of walker 1 and 2 (one cycle = one stride)
mp.sigma   = 0;                    % [rad/s per body weight] load feedback gain
mp.reset   = 1;                    %         1 = CPG phase reset at touchdown (Aoi 2007), 0 = free-running CPG
mp.beta    = 0.69;                 %         nominal duty factor
mp.th_TD   = 2*pi*(1 - mp.beta);   % [rad]   leg phase after touchdown reset
mp.th_on   = mp.th_TD + pi + 0.1;  % [rad]   push-off starts (just after the other leg lands)
mp.th_ramp = 0.6;                  % [rad]   push-off ramp
mp.th_hold = pi - 0.3;             % [rad]   push-off held this long after th_on

% Foot-mass model (mp.legmodel = 2)
% ---------------------------------
mp.legmodel = 1;       %         1 = leg damper, 2 = point-mass foot (set by legmodel below)
mp.mf    = 1.0;        % [kg]    foot mass, each leg (placeholder, not tuned to data)
mp.Ksw   = 1e4;        % [N/m]   hip servo stiffness pulling the swing foot to its target
mp.Dsw   = 140;        % [Ns/m]  hip servo damping (about 0.7 of critical for mf = 1 kg)
mp.h_sw  = 0.08;       % [m]     foot lift at mid-swing (ground clearance)

% Model options
% -------------
mp.flight  = 0;        %         1 = allow a flight phase (running presets); 0 = flight is a fall (walking)
mp.nW      = 2;        %         number of walkers (1 or 2)
mp.coupled = 1;        %         1 = arms + object active
mp.z_fail  = 0.5;      % [l0]    CoM below 0.5 l0 = fall
mp.dt      = 1e-4;     % [s]     gait-logic update step (= integrator step)
mp.dt_anim = 0.04;     % [s]     animation frame interval (must be a multiple of mp.dt)
mp.movie   = 0;        %         1 = also write cpg_pair.mp4
mp.gif     = 0;        %         1 = also write cpg_pair.gif

% Initial conditions: both walkers at an apex of single support on leg 1
% ----------------------------------------------------------------------
y0 = 0.9708;           % [m]   apex height of the single-walker fixed point (Stage 1a result)
v_set = [];            % [m/s] apex speed set directly (running presets); [] = from the energy E0
dE = 5;                % [J]   walker 1 gets E0 - dE, walker 2 gets E0 + dE
E0 = 816;              % [J]   Geyer 2006 Fig. 3A
D  = 2.0;              % [m]   initial distance walker 1 -> walker 2

switch preset
    case 'stage1_pair'
        % passive pair: defaults above
    case 'stage2_single'
        mp.nW = 1;  mp.coupled = 0;  dE = 0;
        mp.cpg = 1;  mp.c = 100;  mp.dl = 0.01;
    case 'stage3_pair'
        mp.cpg = 1;  mp.c = 100;  mp.dl = 0.01;
        mp.omega = 2*pi*[1 1];         % same CPG clock for both walkers (2*pi*[1 1.05] = mismatch)
        dE = 0;                        % same start energy for both walkers
    case {'stage2_run', 'stage3_run'}
        % RUNNING (exploratory): spring-mass runner of Seyfarth et al. 2002 / Geyer 2006 Fig. 3B
        % (k = 20 kN/m, a0 = 68 deg, 5 m/s, apex in flight), leg damper + CPG push-off from
        % mid-stance, flight phase allowed.  Start at a flight apex, leg 1 lands first.
        % The CPG numbers below are placeholders (stride 0.7 s, duty factor 0.3), not fitted.
        mp.k = 20e3;  mp.a0 = 68*pi/180;  mp.flight = 1;
        mp.cpg = 1;  mp.c = 100;  mp.dl = 0.02;
        mp.beta = 0.3;  mp.th_TD = 2*pi*(1 - mp.beta);
        mp.th_on = mp.th_TD + 0.5*2*pi*mp.beta;   % push-off starts half way through stance
        mp.th_ramp = 0.4;  mp.th_hold = 2.0;
        mp.omega = 2*pi/0.7*[1 1];
        y0 = 1.00;  v_set = 5.0;  dE = 0;         % flight apex height and speed (Seyfarth 2002)
        if strcmp(preset, 'stage2_run'), mp.nW = 1;  mp.coupled = 0; end
    otherwise
        error('unknown preset %s', preset);
end
phi_start = 3*pi/2;                    % CPG phase at t = 0 (leg 1 in stance)
if mp.flight > 0.5, phi_start = mp.th_TD - 1.1; end   % running: leg 1 lands about 0.12 s after the apex
if ~isempty(lag0), dE = 0; end         % lag tests: both walkers start with their own fixed-point energy
if ~isempty(dE_set), dE = dE_set; end  % explicit choice
if exist('dl_set', 'var') && ~isempty(dl_set), mp.dl = dl_set; end   % push-off override [m]
if exist('sigma_set', 'var') && ~isempty(sigma_set), mp.sigma = sigma_set; end   % load-feedback gain override
if exist('reset_set', 'var') && ~isempty(reset_set), mp.reset = reset_set; end   % touchdown reset on/off

switch legmodel
    case 'damper'
        % as set by the preset
    case 'footmass'
        if mp.cpg > 0.5                % Stage 2 and 3: swap the leg damper for foot masses
            mp.legmodel = 2;
            mp.c  = 0;                 % no leg damper: the foot impacts are the sink
            mp.dl = 0.003;             % [m]  push-off (replaces the foot losses, about 1 J per step)
            mp.omega = 2*pi/0.7*[1 1];  % [rad/s] swing must finish before the foot lands
            phi_start = 3.6;           % leg 2 half way through its swing at t = 0
        else
            fprintf('cpg_pair_params: preset %s is the passive Lanini model, legmodel ignored\n', preset);
        end
    otherwise
        error('unknown legmodel %s (use ''damper'' or ''footmass'')', legmodel);
end

% Bodies of walker 1 and 2, and per-walker parameters by dynamic similarity
% -------------------------------------------------------------------------
switch pairtype
    case 'identical'
        body_m = [80 80];      body_l = [1.0 1.0];
    case 'different'           % Lanini 2017 Table 1/2, Pair 05: behind = subject 5, front = subject 3
        body_m = [57.5 80];    body_l = [0.88 0.94];
    otherwise
        error('unknown pairtype %s (use ''identical'' or ''different'')', pairtype);
end
sm = body_m/mp.m;  sl = body_l/mp.l0;     % mass and length ratios to the 80 kg / 1 m walker
mp.k     = mp.k*sm./sl;                   % k ~ m g / l0
mp.c     = mp.c*sm./sqrt(sl);             % c ~ m sqrt(g / l0)
mp.dl    = mp.dl*sl;                      % push-off ~ l0
mp.mf    = mp.mf*sm;                      % foot mass ~ m
mp.Ksw   = mp.Ksw*sm./sl;
mp.Dsw   = mp.Dsw*sm./sqrt(sl);
mp.h_sw  = mp.h_sw*sl;
mp.omega = mp.omega./sqrt(sl);            % CPG frequency ~ sqrt(g / l0)
mp.a0    = mp.a0*[1 1];
mp.m     = body_m;  mp.l0 = body_l;
y0i = y0*sl;                              % apex height
Ei  = [E0 - dE, E0 + dE].*sm.*sl;         % energy ~ m g l0
if strcmp(pairtype, 'identical')          % make sure nothing differs between the walkers
    for f = {'m', 'l0', 'k', 'a0', 'c', 'dl', 'mf', 'Ksw', 'Dsw', 'h_sw', 'omega'}
        assert(mp.(f{1})(1) == mp.(f{1})(2), 'identical pair: %s differs between the walkers', f{1});
    end
end
% H2: CPG clock mismatch on otherwise identical walkers (omega2 = omega_ratio_set * omega1)
if exist('omega_ratio_set', 'var') && ~isempty(omega_ratio_set), mp.omega(2) = omega_ratio_set*mp.omega(1); end

% apex speed from the energy (Geyer 2006 App. A(b)), leg vertical at the apex
vapex = @(E, i) sqrt(2*E/mp.m(i) - mp.k(i)/mp.m(i)*(mp.l0(i) - y0i(i))^2 - 2*mp.g*y0i(i));
v1 = vapex(Ei(1), 1);  v2 = vapex(Ei(2), 2);
if ~isempty(v_set), v1 = v_set;  v2 = v_set; end

ic = [];
ic.pos  = [0; y0i(1); D; y0i(2); D/2];    % [x1 z1 x2 z2 xo]
ic.vel  = [v1; 0; v2; 0; (v1 + v2)/2];    % [dx1 dz1 dx2 dz2 dxo]
ic.foot = [0; D];                         % stance (leg 1) foot of walker 1 and 2
ic.phi  = phi_start*[1; 1];               % CPG phase
if exist('phi0_set', 'var') && ~isempty(phi0_set), ic.phi = phi0_set(:); end   % initial CPG phases [phi1 phi2]
mp.foot0 = ic.foot.';                     % read by the gait-logic block at t = 0
mp.phi0  = ic.phi.';
mp.init  = zeros(1, 50);                  % start state from sp_start_state (when lag0 is set)
mp.init_use = [0 0];
mp.L0    = [ic.pos(5) - ic.pos(1), ic.pos(3) - ic.pos(5)];   % arm rest lengths (arms start relaxed)
if mp.flight > 0.5, [ic, mp] = sp_flight_start(mp, ic); end  % running: start at a flight apex

% Start with a chosen stride lag and stance legs (each walker walks alone first, then the pair starts)
mp.lag0 = NaN;                            % for the animation title only
mp.stance0 = [1 1];
if ~isempty(lag0) || any(stance0 ~= 1)
    if isempty(lag0), lag0 = 0; end
    mp.lag0 = lag0;
    [ic, mp] = sp_start_state(mp, ic, lag0, stance0);
end

% Simulation
% ----------
tmax     = 30;         % [s]   simulation time
dtsim    = mp.dt;      % [s]   fixed step (ode4)
rtf_anim = 1;          %       animation speed: 1 = real time, 0.5 = slow motion

lm = {'leg damper', 'foot mass'};
if isempty(lag0), lagtxt = 'in step at an apex';
else, lagtxt = sprintf('initial lag %g deg, stance legs [%d %d]', lag0, stance0(1), stance0(2)); end
fprintf('cpg_pair_params: preset ''%s'', %s, %s walkers, dE = %g J, %s (walkers: %d, coupled: %d, CPG: %d)\n', ...
        preset, lm{mp.legmodel}, pairtype, dE, lagtxt, mp.nW, mp.coupled, mp.cpg);
