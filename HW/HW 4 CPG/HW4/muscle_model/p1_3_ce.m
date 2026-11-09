%% Problem 1.3 - Complete contractile element (CE) model in Simulink
% Builds p1_3_ce.slx: a CE subsystem (see build_CE.m) with
%     F_CE = Fmax * fl(lCE) * fv(vCE) * A
% and a small test harness: S from Problem 1.2, lCE and vCE held constant.
%
% Run as a script from the muscle_model folder (needs build_CE.m and the
% lecture functions CE_force_length_relation.m / CE_force_velocity_relation.m).

clear; close all; clc;

%% Parameters (read by the Simulink blocks from the base workspace)
mp.Fmax = 6000;          % [N]   max isometric force
mp.lopt = 0.08;          % [m]   optimum CE length
mp.vmax = 12 * mp.lopt;  % [m/s] max contraction velocity
mp.w    = 0.56;          % [lopt] force-length width
mp.c    = 0.05;          % [ ]   residual force at lopt*(1 +/- w)
mp.N    = 1.5;           % [ ]   eccentric force enhancement
mp.K    = 5;             % [ ]   force-velocity curvature
mp.tauR = 0.030;         % [s]   activation time constant
mp.tauF = 0.080;         % [s]   deactivation time constant
mp.A0   = 0;             % [ ]   initial activation
Tsim    = 1;             % [s]

%% Build the model
mdl = 'p1_3_ce';
if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist([mdl '.slx'], 'file'), delete([mdl '.slx']); end
new_system(mdl); open_system(mdl);

add_block('simulink/Sources/Sine Wave', [mdl '/Stimulation S'], 'SineType', 'Time based', ...
    'Amplitude', '0.5', 'Bias', '0.5', 'Frequency', '2*pi', 'Position', [40 40 80 80]);
add_block('simulink/Sources/Constant', [mdl '/lCE'], 'Value', 'lce_test', 'Position', [40 120 80 150]);
add_block('simulink/Sources/Constant', [mdl '/vCE'], 'Value', 'vce_test', 'Position', [40 190 80 220]);

build_CE([mdl '/CE']);
set_param([mdl '/CE'], 'Position', [180 40 300 220]);

add_block('simulink/Sinks/To Workspace', [mdl '/F_log'], 'VariableName', 'F_log', 'SaveFormat', 'Timeseries', 'Position', [400 60 460 80]);
add_block('simulink/Sinks/To Workspace', [mdl '/A_log'], 'VariableName', 'A_log', 'SaveFormat', 'Timeseries', 'Position', [400 150 460 170]);
add_block('simulink/Sinks/Scope', [mdl '/Scope'], 'Position', [400 220 430 250]);

add_line(mdl, 'Stimulation S/1', 'CE/1', 'autorouting', 'on');
add_line(mdl, 'lCE/1',           'CE/2', 'autorouting', 'on');
add_line(mdl, 'vCE/1',           'CE/3', 'autorouting', 'on');
add_line(mdl, 'CE/1', 'F_log/1', 'autorouting', 'on');
add_line(mdl, 'CE/1', 'Scope/1', 'autorouting', 'on');
add_line(mdl, 'CE/2', 'A_log/1', 'autorouting', 'on');

set_param(mdl, 'StopTime', num2str(Tsim), 'Solver', 'ode45', 'MaxStep', '1e-3', ...
    'RelTol', '1e-6', 'ReturnWorkspaceOutputs', 'on');
save_system(mdl);

%% Test 1: isometric at optimum length  ->  F_CE should equal Fmax * A
lce_test = mp.lopt;  vce_test = 0;
out1 = sim(mdl);
t  = out1.F_log.Time;  F1 = squeeze(out1.F_log.Data);  A1 = squeeze(out1.A_log.Data);

%% Test 2: longer than optimum and shortening  ->  F_CE scaled by fl*fv
lce_test = 1.2 * mp.lopt;  vce_test = -0.2 * mp.vmax;
out2 = sim(mdl);
F2 = squeeze(out2.F_log.Data);

%% Plot
figure('Name', 'CE force', 'Position', [100 100 900 350]);
subplot(1,2,1);
plot(t, 0.5*sin(2*pi*t) + 0.5, 'k--', 'LineWidth', 1.3); hold on;
plot(t, A1, 'LineWidth', 2); grid on;
xlabel('time [s]'); ylabel('[ ]'); legend({'S', 'A'}); title('Input and activation');

subplot(1,2,2);
plot(t, F1, 'LineWidth', 2); hold on; plot(t, F2, 'LineWidth', 2);
yline(mp.Fmax, ':k'); grid on; ylim([0 6600]);
xlabel('time [s]'); ylabel('F_{CE} [N]'); title('CE force');
legend({'isometric at l_{opt}', 'l_{CE} = 1.2 l_{opt}, v_{CE} = -0.2 v_{max}'}, 'Location', 'northeast');

%% Checks
M2 = CE_force_length_relation(1.2, mp) * CE_force_velocity_relation(-0.2, mp);
fprintf('Test 1: max F_CE = %.0f N  (expected Fmax * max A = %.0f N)\n', max(F1), mp.Fmax * max(A1));
fprintf('Test 2: fl*fv = %.3f, max F_CE = %.0f N  (expected %.0f N)\n', M2, max(F2), mp.Fmax * M2 * max(A1));

%% Export figure for the Overleaf write-up
if ~exist('figures', 'dir'), mkdir('figures'); end
exportgraphics(gcf, 'figures/p1_3_FCE.png', 'Resolution', 300);
