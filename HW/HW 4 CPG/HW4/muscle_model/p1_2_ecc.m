%% Problem 1.2 - Excitation-contraction coupling in Simulink
% Builds the Simulink model p1_2_ecc.slx programmatically, simulates 1 s,
% and plots the neural stimulation S(t) and muscle activation A(t).
%
% Model:  dA/dt = (S - A) / tau,   tau = tau_R  if S > A  (activation, S "rising")
%                                  tau = tau_F  if S < A  (deactivation, S "falling")
%         S(t)  = 0.5 sin(2*pi*t) + 0.5
%
% Run this file as a script (it uses the base workspace variable mp).

clear; close all; clc;

%% Parameters
mp.tauR = 0.030;   % [s] rise (activation) time constant
mp.tauF = 0.080;   % [s] fall (deactivation) time constant
mp.A0   = 0;       % [ ] initial activation (muscle at rest)
Tsim    = 1;       % [s] simulation time

%% Build the Simulink model
mdl = 'p1_2_ecc';
if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist([mdl '.slx'], 'file'), delete([mdl '.slx']); end
new_system(mdl);
open_system(mdl);

% Neural stimulation S = 0.5 sin(2 pi t) + 0.5
add_block('simulink/Sources/Sine Wave', [mdl '/Stimulation S'], ...
    'SineType', 'Time based', 'Amplitude', '0.5', 'Bias', '0.5', ...
    'Frequency', '2*pi', 'Phase', '0', 'Position', [40 100 80 140]);

% Error S - A
add_block('simulink/Math Operations/Sum', [mdl '/S - A'], ...
    'Inputs', '+-', 'Position', [150 105 180 135]);

% Choose 1/tau: 1/tauR if (S - A) > 0, otherwise 1/tauF
add_block('simulink/Sources/Constant', [mdl '/1 over tauR'], ...
    'Value', '1/mp.tauR', 'Position', [150 30 210 60]);
add_block('simulink/Sources/Constant', [mdl '/1 over tauF'], ...
    'Value', '1/mp.tauF', 'Position', [150 190 210 220]);
add_block('simulink/Signal Routing/Switch', [mdl '/rising or falling'], ...
    'Criteria', 'u2 > Threshold', 'Threshold', '0', ...
    'Position', [260 85 300 155]);

% dA/dt = (S - A) / tau
add_block('simulink/Math Operations/Product', [mdl '/times 1 over tau'], ...
    'Inputs', '2', 'Position', [350 100 380 160]);

% Integrator -> A
add_block('simulink/Continuous/Integrator', [mdl '/Integrator'], ...
    'InitialCondition', 'mp.A0', 'Position', [430 115 460 145]);

% Outputs: scope + workspace logging
add_block('simulink/Sinks/Scope', [mdl '/Scope S and A'], ...
    'Position', [560 40 590 90]);
try
    set_param([mdl '/Scope S and A'], 'NumInputPorts', '2');
catch
    set_param([mdl '/Scope S and A'], 'NumInputPorts', 2);
end
add_block('simulink/Sinks/To Workspace', [mdl '/S_log'], ...
    'VariableName', 'S_log', 'SaveFormat', 'Timeseries', ...
    'Position', [560 180 620 200]);
add_block('simulink/Sinks/To Workspace', [mdl '/A_log'], ...
    'VariableName', 'A_log', 'SaveFormat', 'Timeseries', ...
    'Position', [560 230 620 250]);

% Wiring
add_line(mdl, 'Stimulation S/1',      'S - A/1',             'autorouting', 'on');
add_line(mdl, 'S - A/1',              'rising or falling/2', 'autorouting', 'on');
add_line(mdl, 'S - A/1',              'times 1 over tau/1',  'autorouting', 'on');
add_line(mdl, '1 over tauR/1',        'rising or falling/1', 'autorouting', 'on');
add_line(mdl, '1 over tauF/1',        'rising or falling/3', 'autorouting', 'on');
add_line(mdl, 'rising or falling/1',  'times 1 over tau/2',  'autorouting', 'on');
add_line(mdl, 'times 1 over tau/1',   'Integrator/1',        'autorouting', 'on');
add_line(mdl, 'Integrator/1',         'S - A/2',             'autorouting', 'on');  % feedback
add_line(mdl, 'Integrator/1',         'Scope S and A/2',     'autorouting', 'on');
add_line(mdl, 'Integrator/1',         'A_log/1',             'autorouting', 'on');
add_line(mdl, 'Stimulation S/1',      'Scope S and A/1',     'autorouting', 'on');
add_line(mdl, 'Stimulation S/1',      'S_log/1',             'autorouting', 'on');

% Solver settings
set_param(mdl, 'StopTime', num2str(Tsim), 'Solver', 'ode45', ...
    'MaxStep', '1e-3', 'RelTol', '1e-6', 'ReturnWorkspaceOutputs', 'on');
save_system(mdl);

%% Simulate
out = sim(mdl);
tS = out.S_log.Time;  S = squeeze(out.S_log.Data);
tA = out.A_log.Time;  A = squeeze(out.A_log.Data);

%% Plot
figure('Name', 'S and A', 'Position', [100 100 800 380]);
plot(tS, S, 'k--', 'LineWidth', 1.5); hold on;
plot(tA, A, 'LineWidth', 2);
grid on; ylim([-0.05 1.1]);
xlabel('time [s]'); ylabel('[ ]');
legend({'stimulation S', 'activation A'}, 'Location', 'northeast');
title('Excitation-contraction coupling (\tau_R = 30 ms, \tau_F = 80 ms)');

%% Key numbers for the discussion
[Amax, iMax] = max(A .* (tA < 0.6));
mask = tA > 0.5;  [Amin, iMin] = min(A(mask));  tMin = tA(mask);
fprintf('A peak  = %.3f at t = %.3f s (S peaks at 0.25 s -> lag %.0f ms)\n', ...
    Amax, tA(iMax), 1000*(tA(iMax) - 0.25));
fprintf('A trough = %.3f at t = %.3f s (S min at 0.75 s -> lag %.0f ms)\n', ...
    Amin, tMin(iMin), 1000*(tMin(iMin) - 0.75));

%% Export figure for the Overleaf write-up
if ~exist('figures', 'dir'), mkdir('figures'); end
exportgraphics(gcf, 'figures/p1_2_SA.png', 'Resolution', 300);
