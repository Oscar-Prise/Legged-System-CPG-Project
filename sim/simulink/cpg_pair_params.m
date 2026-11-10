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
% Initial lag:
%   lag0 = []    both walkers start at an apex on leg 1 at t = 0 (in step)
%   lag0 = 90    each walker first walks alone for 2 strides (sp_start_state.m), walker 2 a further
%                (360 - lag0)/360 of a stride, so the pair starts with a stride lag of about lag0 deg
%
% Example:  preset = 'stage3_pair'; legmodel = 'damper'; pairtype = 'different'; lag0 = 90;
%           cpg_pair_params
%

if ~exist('preset', 'var') || isempty(preset), preset = 'stage1_pair'; end
if ~exist('legmodel', 'var') || isempty(legmodel), legmodel = 'damper'; end
if ~exist('pairtype', 'var') || isempty(pairtype), pairtype = 'identical'; end
if ~exist('lag0', 'var'), lag0 = []; end

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
        mp.omega = 2*pi*[1 1];         % try e.g. 2*pi*[1 1.05] for a frequency mismatch
    otherwise
        error('unknown preset %s', preset);
end
phi_start = 3*pi/2;                    % CPG phase at t = 0 (leg 1 in stance)
if ~isempty(lag0), dE = 0; end         % lag tests: both walkers start with their own fixed-point energy

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

% apex speed from the energy (Geyer 2006 App. A(b)), leg vertical at the apex
vapex = @(E, i) sqrt(2*E/mp.m(i) - mp.k(i)/mp.m(i)*(mp.l0(i) - y0i(i))^2 - 2*mp.g*y0i(i));
v1 = vapex(Ei(1), 1);  v2 = vapex(Ei(2), 2);

ic = [];
ic.pos  = [0; y0i(1); D; y0i(2); D/2];    % [x1 z1 x2 z2 xo]
ic.vel  = [v1; 0; v2; 0; (v1 + v2)/2];    % [dx1 dz1 dx2 dz2 dxo]
ic.foot = [0; D];                         % stance (leg 1) foot of walker 1 and 2
ic.phi  = phi_start*[1; 1];               % CPG phase
mp.foot0 = ic.foot.';                     % read by the gait-logic block at t = 0
mp.phi0  = ic.phi.';
mp.init  = zeros(1, 50);                  % start state from sp_start_state (when lag0 is set)
mp.init_use = [0 0];
mp.L0    = [ic.pos(5) - ic.pos(1), ic.pos(3) - ic.pos(5)];   % arm rest lengths (arms start relaxed)

% Start with a chosen stride lag (each walker walks alone first, then the pair starts)
mp.lag0 = NaN;                            % for the animation title only
if ~isempty(lag0)
    mp.lag0 = lag0;
    [ic, mp] = sp_start_state(mp, ic, lag0);
end

% Simulation
% ----------
tmax     = 30;         % [s]   simulation time
dtsim    = mp.dt;      % [s]   fixed step (ode4)
rtf_anim = 1;          %       animation speed: 1 = real time, 0.5 = slow motion

lm = {'leg damper', 'foot mass'};
if isempty(lag0), lagtxt = 'in step at an apex'; else, lagtxt = sprintf('initial lag %g deg', lag0); end
fprintf('cpg_pair_params: preset ''%s'', %s, %s walkers, %s (walkers: %d, coupled: %d, CPG: %d)\n', ...
        preset, lm{mp.legmodel}, pairtype, lagtxt, mp.nW, mp.coupled, mp.cpg);
