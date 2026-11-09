%
% m07_params_A2.m
% Assignment 2 - 3-D LIPM Figure-Eight Walking
%

mp = [];

%% Ground
gnd_rng = 20;  % [m]
tile_size = [0.25 0.25];

num_xpnts = gnd_rng / tile_size(1);
num_ypnts = gnd_rng / tile_size(2);

gnd_xpnts = linspace(-gnd_rng/2, gnd_rng/2, num_xpnts);
gnd_ypnts = linspace(-gnd_rng/2, gnd_rng/2, num_ypnts);
gnd_zelev = zeros(num_xpnts, num_ypnts);

%% Robot / LIPM parameters
mp.m = 100;       % body mass [kg]
mp.g = 9.81;      % gravity [m/s^2]
mp.z0 = 0.8;      % constant CoM height [m]
mp.Lmax = 0.9;    % maximum leg length [m]

mp.B = 100000.0;
mp.K = 0.01 * mp.B^2 / mp.m;

%% Walking parameters
mp.T_st  = 0.7;   % stance duration [s]
mp.lstep = 0.5;   % nominal step length [m]
mp.wstep = 0.3;   % nominal step width [m]
mp.vtgt  = 0.9;   % target speed [m/s]

% Foot-placement cost
mp.a = 2;
mp.b = 1;

%% Initial conditions
ic = [];

ic.x0 = 0;
ic.y0 = 0;
ic.z0 = mp.z0;

ic.dx0 = 0;
ic.dy0 = -0.3;
ic.dz0 = 0;

RxFP0 = 0;
RyFP0 = -0.1;
RzFP0 = 0;

LxFP0 = 0;
LyFP0 = mp.wstep;
LzFP0 = 0;

%% Generate requested walking pattern
% mode = 'left_quarter';
% mode = 'right_quarter';
mode = 'figure8';

goal_table = make_A2_goals(mode, mp);

tmax = size(goal_table,1) * mp.T_st;

%% Simulation
dtsim = 1e-4;
rtf_anim = 1;
