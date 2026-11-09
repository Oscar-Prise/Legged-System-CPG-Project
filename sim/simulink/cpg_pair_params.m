%
% cpg_pair_params.m -- Parameters for the Simulink model cpg_pair.slx
%
% Two 2-D spring-mass walkers (Geyer, Seyfarth & Blickhan 2006) holding an object
% through horizontal spring-damper arms (Lanini et al. 2017, Sec. 3.2), each with a
% CPG-timed push-off (Owaki & Ishiguro 2017 Eq. 1, reset at touchdown as in
% Aoi & Tsuchiya 2007).  Same equations as ../slip_*.m (the ode45 reference code).
%
% Choose a preset BEFORE running this script, e.g.
%     preset = 'stage2_single';  cpg_pair_params
% then press Run in the model.  Without a preset, 'stage1_pair' is used.
%
%   'stage1_pair'    passive Lanini pair (no CPG, no leg damper)          Stage 1
%   'stage2_single'  one walker with leg damper + CPG push-off             Stage 2
%   'stage3_pair'    two CPG walkers coupled by the object (first look)    Stage 3
%

if ~exist('preset', 'var') || isempty(preset), preset = 'stage1_pair'; end

mp = [];

% Walker (Geyer 2006 Fig. 3A)
% ---------------------------
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

% Model options
% -------------
mp.nW      = 2;        %         number of walkers (1 or 2)
mp.coupled = 1;        %         1 = arms + object active
mp.z_fail  = 0.5;      % [m]     CoM below this = fall
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

% apex speed from the energy (Geyer 2006 App. A(b)), leg vertical at the apex
vapex = @(E) sqrt(2*E/mp.m - mp.k/mp.m*(mp.l0 - y0)^2 - 2*mp.g*y0);
v1 = vapex(E0 - dE);  v2 = vapex(E0 + dE);

ic = [];
ic.pos  = [0; y0; D; y0; D/2];            % [x1 z1 x2 z2 xo]
ic.vel  = [v1; 0; v2; 0; (v1 + v2)/2];    % [dx1 dz1 dx2 dz2 dxo]
ic.foot = [0; D];                         % stance (leg 1) foot of walker 1 and 2
ic.phi  = 3*pi/2*[1; 1];                  % CPG phase (leg 1 in mid-stance)
mp.foot0 = ic.foot.';                     % read by the gait-logic block at t = 0
mp.phi0  = ic.phi.';
mp.L0    = [ic.pos(5) - ic.pos(1), ic.pos(3) - ic.pos(5)];   % arm rest lengths (arms start relaxed)

% Simulation
% ----------
tmax     = 30;         % [s]   simulation time
dtsim    = mp.dt;      % [s]   fixed step (ode4)
rtf_anim = 1;          %       animation speed: 1 = real time, 0.5 = slow motion

fprintf('cpg_pair_params: preset ''%s'' loaded (walkers: %d, coupled: %d, CPG: %d)\n', ...
        preset, mp.nW, mp.coupled, mp.cpg);
