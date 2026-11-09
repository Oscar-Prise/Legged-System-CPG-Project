%% Problem 2.2 - Extensor-flexor pair: a Matsuoka half-center oscillator
% Two neurons from build_neuron.m inhibit each other:
%     e receives  w_fe * y_f      (w_fe = -1.5)
%     f receives  w_ef * y_e      (w_ef = -1.5)
% with x_e(0) = 1 so the symmetric pair has something to break the tie.
% Then: sweep tau_a (what happens below 2*tau_x?) and the weight imbalance.
% Run from the cpg_model folder.

clear; close all; clc;

%% Parameters (read by the Simulink blocks from the base workspace)
np.x0   = 5;     np.taux = 1;     np.taua = 12;    np.wp = 2.5;
np.wfe  = -1.5;  % weight from flexor to extensor (input to e)
np.wef  = -1.5;  % weight from extensor to flexor (input to f)
Tsim    = 100;

%% Build the model
mdl = 'p2_2_oscillator';
if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist([mdl '.slx'], 'file'), delete([mdl '.slx']); end
new_system(mdl); open_system(mdl);

add_block('simulink/Sources/Constant', [mdl '/x0'], 'Value', 'np.x0', 'Position', [40 120 80 150]);
build_neuron([mdl '/e_L'], '1');     % extensor, x_e(0) = 1
build_neuron([mdl '/f_L'], '0');     % flexor,   x_f(0) = 0
set_param([mdl '/e_L'], 'Position', [220 30 320 130]);
set_param([mdl '/f_L'], 'Position', [220 190 320 290]);

% synaptic weights live OUTSIDE the neurons, on the wires between them
add_block('simulink/Math Operations/Gain', [mdl '/w_fe'], 'Gain', 'np.wfe', 'Orientation', 'left', 'Position', [400 150 440 180]);
add_block('simulink/Math Operations/Gain', [mdl '/w_ef'], 'Gain', 'np.wef', 'Orientation', 'left', 'Position', [400 90 440 120]);

add_line(mdl, 'x0/1', 'e_L/1', 'autorouting', 'on');     % tonic drive to both
add_line(mdl, 'x0/1', 'f_L/1', 'autorouting', 'on');
add_line(mdl, 'f_L/1', 'w_fe/1', 'autorouting', 'on');   % y_f * w_fe -> e
add_line(mdl, 'w_fe/1', 'e_L/2', 'autorouting', 'on');
add_line(mdl, 'e_L/1', 'w_ef/1', 'autorouting', 'on');   % y_e * w_ef -> f
add_line(mdl, 'w_ef/1', 'f_L/2', 'autorouting', 'on');

add_block('simulink/Sinks/To Workspace', [mdl '/ye_log'], 'VariableName', 'ye_log', 'SaveFormat', 'Timeseries', 'Position', [520 40 580 60]);
add_block('simulink/Sinks/To Workspace', [mdl '/yf_log'], 'VariableName', 'yf_log', 'SaveFormat', 'Timeseries', 'Position', [520 260 580 280]);
add_block('simulink/Sinks/Scope', [mdl '/Scope'], 'Position', [520 140 550 170]);
set_param([mdl '/Scope'], 'NumInputPorts', '2');
add_line(mdl, 'e_L/1', 'ye_log/1', 'autorouting', 'on');
add_line(mdl, 'f_L/1', 'yf_log/1', 'autorouting', 'on');
add_line(mdl, 'e_L/1', 'Scope/1',  'autorouting', 'on');
add_line(mdl, 'f_L/1', 'Scope/2',  'autorouting', 'on');

set_param(mdl, 'StopTime', num2str(Tsim), 'Solver', 'ode45', 'MaxStep', '0.01', ...
    'RelTol', '1e-6', 'ReturnWorkspaceOutputs', 'on');
save_system(mdl);

%% (a) Baseline
[t, ye, yf] = run_osc(mdl);
figure('Name', 'Oscillator', 'Position', [100 100 1000 640]);
subplot(2,2,1); hold on; grid on;
plot(t, ye, 'LineWidth', 2); plot(t, yf, 'LineWidth', 2); xlim([0 60]);
xlabel('time [s]'); ylabel('output'); title('Baseline (w_{fe} = w_{ef} = -1.5)'); legend({'y_e', 'y_f'});
r = osc_stats(t, ye, 40);
fprintf('Baseline: period %.1f s, amplitude %.2f, extensor active %.0f%% of the period\n', r.period, r.amp, 100*r.duty);

%% (b) Sweep tau_a: period and amplitude; oscillation should die below 2*tau_x
taus = [1 1.5 1.8 2 2.2 2.5 3 4 6 8 12 20 30 50];
per = nan(size(taus));  amp = nan(size(taus));
for k = 1:numel(taus)
    np.taua = taus(k);
    set_param(mdl, 'StopTime', num2str(max(150, 8*taus(k))));
    [t, ye] = run_osc(mdl);
    r = osc_stats(t, ye, 0.4*t(end));
    if r.osc, per(k) = r.period; amp(k) = r.amp; end
    fprintf('tau_a = %4.1f : %s\n', taus(k), ternary(r.osc, sprintf('period %.1f s, amp %.2f', r.period, r.amp), 'no oscillation'));
end
np.taua = 12;  set_param(mdl, 'StopTime', num2str(Tsim));
subplot(2,2,2); hold on; grid on;
yyaxis left;  plot(taus, per, 'o-', 'LineWidth', 1.8); ylabel('period [s]');
yyaxis right; plot(taus, amp, 's--', 'LineWidth', 1.5); ylabel('amplitude of y_e'); ylim([0 3.5]);
set(gca, 'XScale', 'log'); xline(2, '--k', '\tau_a = 2\tau_x'); xlabel('\tau_a [s]'); title('Period vs. adaptation time');

%% (c) Time traces just above and below 2*tau_x
subplot(2,2,3); hold on; grid on;
for taua = [3 1.5]
    np.taua = taua;  [t, ye, yf] = run_osc(mdl);
    ls = ternary(taua > 2, '-', '--');
    plot(t, ye, ls, 'Color', [0 .45 .74], 'LineWidth', 2, 'DisplayName', sprintf('y_e, \\tau_a = %g', taua));
    plot(t, yf, ls, 'Color', [.85 .33 .1], 'LineWidth', 2, 'DisplayName', sprintf('y_f, \\tau_a = %g', taua));
end
np.taua = 12;
xlim([0 40]); xlabel('time [s]'); ylabel('output'); title('\tau_a above vs. below 2\tau_x'); legend('NumColumns', 2);

%% (d) Weight imbalance: extensor inhibits flexor harder
fprintf('\nImbalance (w_fe = -1.5 fixed):\n');
for wef = [-0.5 -1 -1.5 -2 -3]
    np.wef = wef;  [t, ye, yf] = run_osc(mdl);
    r = osc_stats(t, ye, 40);  m = t > 40;
    fprintf('  w_ef = %4.1f : %s, e dominant %.0f%% of the time, peaks y_e %.2f / y_f %.2f\n', wef, ...
        ternary(r.osc, sprintf('period %.1f s', r.period), 'no oscillation'), 100*mean(ye(m) > yf(m)), max(ye(m)), max(yf(m)));
    if wef == -3, te = t; yee = ye; yff = yf; end
end
np.wef = -1.5;
subplot(2,2,4); hold on; grid on;
m = te >= 30;  plot(te(m)-30, yee(m), 'LineWidth', 2); plot(te(m)-30, yff(m), 'LineWidth', 2);
xlim([0 70]); xlabel('time [s]'); ylabel('output'); title('Imbalance: w_{fe} = -1.5, w_{ef} = -3'); legend({'y_e', 'y_f'});

if ~exist('figures', 'dir'), mkdir('figures'); end
exportgraphics(gcf, 'figures/p2_2_oscillator.png', 'Resolution', 300);

%% ---- local functions ---------------------------------------------------
function [t, ye, yf] = run_osc(mdl)
out = sim(mdl);
t = out.ye_log.Time;  ye = squeeze(out.ye_log.Data);  yf = squeeze(out.yf_log.Data);
end

function r = osc_stats(t, y, t0)
% period from successive onsets (y rising through 0), amplitude, duty cycle
m = t > t0;  t = t(m);  y = y(m);
on = y > 1e-6;  onsets = find(diff(on) == 1);
r.osc = numel(onsets) >= 2 && (max(y) - min(y)) > 1e-3;
r.amp = max(y);  r.period = NaN;  r.duty = NaN;
if r.osc
    r.period = mean(diff(t(onsets)));
    r.duty   = mean(on(onsets(1):onsets(end)));
end
end

function out = ternary(c, a, b)
if c, out = a; else, out = b; end
end
