%
% m05_params.m -- Linear Gait Parameter Script
%
% hgeyer, Aug 2025
% 

mp = [];

% Ground Profile
% --------------

gnd_rng = 20;  % [m]
tile_size = [0.25 0.25]; % tile length and width [m]
num_xpnts = gnd_rng / tile_size(1);
num_ypnts = gnd_rng / tile_size(2);

gnd_xpnts = linspace(-gnd_rng / 2, gnd_rng / 2, num_xpnts);
gnd_ypnts = linspace(-gnd_rng / 2, gnd_rng / 2, num_ypnts);
gnd_zelev = zeros(num_xpnts, num_ypnts);


% Model Parameters
% ----------------


% center body
mp.m = 70;    % body mass [kg] 
mp.g = 9.81;  % gravitational acceleration [m/s^2]

% linear gait leg
mp.z0 =  0.8;           % nominal height [m]
mp.B = 100000.0;        % impact damping [Ns/m]
mp.K = 0.01 * mp.B^2 / mp.m;  % impact stiffness [N/m] <1>


% Transitions
% -----------

mp.T_st = 0.7;   % [s]   stance duration
mp.lstep = 0.5;  % [m]  step length
mp.wstep = 0.3;  % [m] step width
mp.vtgt = 0.9;   % [m/s]  target walking speed

mp.a = 2;  % weights terms of foot placement cost function 
mp.b = 1;  % J = a * (x_f - x_d)^2 + b * (dx_f - dx_des)^2


% Initial Conditions
% ------------------

ic = [];

ic.x0 = 0;  %[m]
ic.y0 = 0;  % [m]
ic.z0 = mp.z0; %[m]

% initial velocity
ic.dx0 = 0;  % [m/s]
ic.dy0 = -0.3;  % [m/s]
ic.dz0 = 0.0;  % [m/s]

RxFP0 = 0;  % [m]
RyFP0 = -0.1;  % [m]
RzFP0 = 0;  % [m]

LxFP0 = 0;  % [m]
LyFP0 = mp.wstep;  % [m]
LzFP0 = 0;  % [m]

% Simulation Integrator
% ---------------------

tmax = 3;       % max simulation time
dtsim = 1e-4;   % time step for fixed step integrator
rtf_anim = 1;  % animation realtime factor


% -----------------------------------------------------------------------------------
% <1>: Stiffness should be less than critical stiffness k_crit = B^2 / m
% for overdamped behavior.
