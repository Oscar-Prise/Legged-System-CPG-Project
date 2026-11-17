function T = plot_ic_sweep(files, outname)
% PLOT_IC_SWEEP  Table and figures from one or more sweep_initial_conditions results.
%
%   T = plot_ic_sweep('../results/ic_sweep_damper.mat')
%   T = plot_ic_sweep({'a.mat', 'b.mat'}, '../results/ic_sweep_damper')   % merged, figures saved
%
% Figures: (1) final stride lag vs initial lag per stance-leg combination (lock / drift / fall),
%          (2) lag vs time for every run, (3) physical offset: final vs initial.

if ischar(files), files = {files}; end
S = [];  legmodel = '';  tEnd = NaN;
for f = 1:numel(files)
    L = load(files{f});  S = [S, L.sweep.S];  legmodel = L.sweep.legmodel;  tEnd = L.sweep.tEnd;   %#ok<AGROW>
end
if nargin < 2 || isempty(outname), outname = ''; end
n = numel(S);
st = zeros(n, 2);  lag0 = zeros(n, 1);  lag_end = nan(n, 1);  off0 = zeros(n, 1);  off_end = nan(n, 1);
drift = nan(n, 1);  locked = false(n, 1);  fell = false(n, 1);  label = cell(n, 1);  cls = cell(n, 1);  tend = nan(n, 1);
v1 = nan(n, 1);  T1 = nan(n, 1);  Earm = nan(n, 1);  Wpush = nan(n, 1);
for i = 1:n
    G = S(i).G;
    st(i, :) = S(i).stance0;  lag0(i) = S(i).lag0;
    l0 = mod(lag0(i) + 180*(st(i, 2) ~= st(i, 1)), 360);   % stride lag the labels give at t = 0
    lag0(i) = l0;
    p = mod(l0, 180);  off0(i) = min(p, 180 - p);
    lag_end(i) = G.lag_end;  off_end(i) = G.offset;  drift(i) = G.drift;
    G.locked = ~G.fell && abs(G.drift) < 0.3 && G.lag_std < 5;    % (re)label with the current rule
    [G.label, cls{i}] = sp_gait_label(G);
    locked(i) = G.locked;  fell(i) = G.fell;  label{i} = G.label;  tend(i) = G.t_end;
    v1(i) = G.v1;  T1(i) = G.T1;  Earm(i) = G.Earm;  Wpush(i) = G.Wpush;
end
stance = arrayfun(@(i) sprintf('[%d %d]', st(i, 1), st(i, 2)), 1:n, 'UniformOutput', false).';
T = table(stance, lag0, off0, lag_end, off_end, drift, locked, fell, tend, label, v1, T1, Earm, Wpush);
T.Properties.VariableDescriptions = {'stance legs [w1 w2] at t=0', 'initial stride lag [deg]', ...
    'initial physical offset [deg, 0-90]', 'final stride lag [deg]', 'final physical offset [deg]', ...
    'lag drift over the last 10 s [deg/s]', 'locked', 'fell', 'end time [s]', 'gait label', ...
    'speed walker 1 [m/s]', 'stride time [s]', 'arm loss per stride [J]', 'push-off work per stride [J]'};
disp(T);

% how many gaits
fprintf('\n%s model, %d runs of %g s, two identical walkers\n', legmodel, n, tEnd);
u = unique(cls);
for k = 1:numel(u)
    m = strcmp(cls, u{k});
    fprintf('  %-36s %2d runs   final lag %s\n', u{k}, sum(m), mat2str(round(lag_end(m).'), 3));
end
fprintf('  locked: %d, drifting: %d, fell: %d\n', sum(locked), sum(~locked & ~fell), sum(fell));
if any(locked)
    fprintf('  distinct physical offsets among locked runs (5 deg bins): %s deg\n', ...
            mat2str(unique(5*round(off_end(locked)/5)).'));
end

% ---- figure 1: final lag vs initial lag -----------------------------------------
combos = unique(stance);
cols = lines(numel(combos));
fig1 = figure('Name', 'IC sweep: final vs initial lag', 'Position', [60 60 1000 440], 'Visible', 'on');
subplot(1, 2, 1); hold on; grid on;
for k = 1:numel(combos)
    m = strcmp(stance, combos{k});
    plot(lag0(m & locked), lag_end(m & locked), 'o', 'Color', cols(k, :), 'MarkerFaceColor', cols(k, :), 'DisplayName', [combos{k} ' locked']);
    plot(lag0(m & ~locked & ~fell), lag_end(m & ~locked & ~fell), 's', 'Color', cols(k, :), 'DisplayName', [combos{k} ' drifting']);
    plot(lag0(m & fell), lag_end(m & fell), 'x', 'Color', cols(k, :), 'MarkerSize', 9, 'DisplayName', [combos{k} ' fell']);
end
for r = [0 90 180 225 270 360], yline(r, ':', 'Color', [.5 .5 .5], 'HandleVisibility', 'off'); end
plot([0 360], [0 360], '--', 'Color', [.7 .7 .7], 'HandleVisibility', 'off');
xlabel('initial stride lag (walker 2 vs 1) [deg]'); ylabel('final stride lag, last 10 s [deg]');
title(sprintf('%s model, %g s runs', legmodel, tEnd)); xlim([-5 365]); ylim([-5 365]);
legend('Location', 'eastoutside', 'FontSize', 7, 'NumColumns', 1);
subplot(1, 2, 2); hold on; grid on;
h = [];
if any(locked), h(end+1) = plot(off0(locked), off_end(locked), 'ko', 'MarkerFaceColor', 'k', 'DisplayName', 'locked'); end
if any(~locked & ~fell), h(end+1) = plot(off0(~locked & ~fell), off_end(~locked & ~fell), 'bs', 'DisplayName', 'still drifting'); end
if any(fell), h(end+1) = plot(off0(fell), zeros(sum(fell), 1), 'rx', 'MarkerSize', 9, 'DisplayName', 'fell (plotted at 0)'); end
plot([0 90], [0 90], '--', 'Color', [.7 .7 .7], 'HandleVisibility', 'off');
xlabel('initial physical offset [deg of stride, 0 = in step, 90 = half a step]');
ylabel('final physical offset [deg]'); title('same thing, leg labels folded out');
if ~isempty(h), legend(h, 'Location', 'northwest'); end

% ---- figure 2: lag vs time ---------------------------------------------------------
fig2 = figure('Name', 'IC sweep: lag vs time', 'Position', [60 540 1000 420]);
hold on; grid on;
for i = 1:n
    k = find(strcmp(combos, stance{i}));
    G = S(i).G;
    if isempty(G.tl), continue; end
    plot(G.tl, G.lag, '.-', 'Color', cols(k, :), 'LineWidth', 0.8);
    if G.fell, plot(G.t_end, G.lag(end), 'x', 'Color', cols(k, :), 'MarkerSize', 9); end
end
for r = [0 90 180 225 270 360], yline(r, ':', 'Color', [.5 .5 .5], 'HandleVisibility', 'off'); end
xlabel('time [s]'); ylabel('stride lag, walker 2 vs 1 [deg]'); ylim([-10 370]);
title(sprintf('%s model: stride lag over time, all runs (colour = stance legs %s)', legmodel, strjoin(combos, ', ')));

if ~isempty(outname)
    print(fig1, '-dpng', '-r130', [outname '_final_vs_initial.png']);
    print(fig2, '-dpng', '-r130', [outname '_lag_vs_time.png']);
    writetable(T, [outname '.csv']);
    fprintf('wrote %s_final_vs_initial.png, %s_lag_vs_time.png, %s.csv\n', outname, outname, outname);
end
end
