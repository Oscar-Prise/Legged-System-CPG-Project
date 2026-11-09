%
% Parameter File for muscle model.
%
% hgeyer
% Oct, 2025
%

mp = [];

% SHARED MUSCLE PARAMS

% excitation-contraction coupling
mp.preA =  0.01;  % [] preactivation
mp.tau = 0.01;    % [s] delay time constant

% CE force length relation
mp.w = 0.56;  % [lopt] width
mp.c = 0.05;  % [] remaining force at +/- width

% CE force velocity relation
mp.N = 1.5;  % [Fmax] eccentric force enhancement
mp.K = 5;    % [] shape factor

% SE force length relation
mp.eref = 0.04; %[lslack] tendon reference strain

% MUSCLE-SPECIFIC PARAMETERS

% vasti muscle group
mp.VAS.Fmax = 6000;   % CE maximum isometric force [N]
mp.VAS.lopt = 0.08;    % CE optimum fiber length [m]
mp.VAS.vmax = 12;      % CE maximum contraction velocity [lopt/s]
mp.VAS.lslack = 0.23;  % SE slack length [m]



% simulation parameters
rtf_anim = 0.3;  % animation speed realtime factor
dtsim = 1e-4; 