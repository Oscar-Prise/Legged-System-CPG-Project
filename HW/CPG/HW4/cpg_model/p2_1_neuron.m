%% Problem 2.1 - A single Matsuoka neuron (left extensor) in Simulink
% Builds p2_1_neuron.slx around the neuron subsystem of build_neuron.m,
% simulates the baseline, then sweeps the central input x0 and the
% adaptation time constant tau_a.  Run from the folder that holds build_neuron.m.

clear; close all; clc;

%% Neuron parameters (read by the Simulink blocks from the base workspace)
np.x0   = 5;     % [ ] central (tonic) input
np.taux = 1;     % [s] membrane time constant
np.taua = 12;    % [s] adaptation time constant
np.wp   = 2.5;   % [ ] adaptation weight w'
Tsim    = 40;    % [s]

%% Build the model:  x0 --> [neuron] --> y
mdl = 'p2_1_neuron';
if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist([mdl '.slx'], 'file'), delete([mdl '.slx']); end
new_system(mdl); open_system(mdl);

add_block('simulink/Sources/Constant', [mdl '/x0'],     'Value', 'np.x0', 'Position', [40 60 80 90]);
add_block('simulink/Sources/Constant', [mdl '/no input'], 'Value', '0',     'Position', [40 130 80 160]);  % sum_i w_i y_i = 0
build_neuron([mdl '/e_L']);
set_param([mdl '/e_L'], 'Position', [160 50 260 170]);

logs = {'y_log', 'e_L/1'; 'x_log', 'e_L/2'; 'xa_log', 'e_L/3'};
for k = 1:size(logs, 1)
    add_block('simulink/Sinks/To Workspace', [mdl '/' logs{k,1}], 'VariableName', logs{k,1}, ...
        'SaveFormat', 'Timeseries', 'Position', [360 40+50*k 420 60+50*k]);
    add_line(mdl, logs{k,2}, [logs{k,1} '/1'], 'autorouting', 'on');
end
add_block('simulink/Sinks/Scope', [mdl '/Scope'], 'Position', [360 220 390 250]);
add_line(mdl, 'x0/1',       'e_L/1',   'autorouting', 'on');
add_line(mdl, 'no input/1', 'e_L/2',   'autorouting', 'on');
add_line(mdl, 'e_L/1',      'Scope/1', 'autorouting', 'on');

set_param(mdl, 'StopTime', num2str(Tsim), 'Solver', 'ode45', 'MaxStep', '0.01', ...
    'RelTol', '1e-6', 'ReturnWorkspaceOutputs', 'on');
save_system(mdl);

%% Helper: run the model with the current np and return [t, y, x, xa]
run_it = @() deal_sim(mdl);

%% 1) Baseline
[t, y, x, xa] = run_it();
figure('Name', 'Single neuron', 'Position', [100 100 1200 360]);
subplot(1,3,1); hold on; grid on;
plot(t, x, '--', 'Color', [.5 .5 .5], 'LineWidth', 1.5);
plot(t, xa, 'LineWidth', 1.5); plot(t, y, 'LineWidth', 2.2);
yline(np.x0 / (1 + np.wp), ':k');
xlabel('time [s]'); title(sprintf('Baseline (x_0 = %g, \\tau_a = %g)', np.x0, np.taua));
legend({'membrane x', 'adaptation x''', 'output y'}, 'Location', 'northeast');
fprintf('Baseline: y peaks at %.2f (t = %.1f s), settles at %.3f (predicted x0/(1+w'') = %.3f)\n', ...
        max(y), t(find(y == max(y), 1)), y(end), np.x0 / (1 + np.wp));

%% 2) Sweep central input x0
subplot(1,3,2); hold on; grid on;
for x0 = [-2 1 5 10]
    np.x0 = x0;  [t, y] = run_it();
    plot(t, y, 'LineWidth', 2, 'DisplayName', sprintf('x_0 = %g', x0));
end
np.x0 = 5;
xlabel('time [s]'); ylabel('y'); title('Effect of x_0 (\tau_a = 12)'); legend('Location', 'northeast');

%% 3) Sweep adaptation time constant tau_a
subplot(1,3,3); hold on; grid on;
for taua = [1 3 12 50]
    np.taua = taua;  [t, y] = run_it();
    plot(t, y, 'LineWidth', 2, 'DisplayName', sprintf('\\tau_a = %g', taua));
end
np.taua = 12;
xlabel('time [s]'); ylabel('y'); title('Effect of \tau_a (x_0 = 5)'); legend('Location', 'northeast');

%% Export figure for the Overleaf write-up
if ~exist('figures', 'dir'), mkdir('figures'); end
exportgraphics(gcf, 'figures/p2_1_neuron.png', 'Resolution', 300);

%% ---- local function: simulate and unpack the logged signals ----------
function [t, y, x, xa] = deal_sim(mdl)
out = sim(mdl);
t  = out.y_log.Time;
y  = squeeze(out.y_log.Data);
x  = squeeze(out.x_log.Data);
xa = squeeze(out.xa_log.Data);
end
