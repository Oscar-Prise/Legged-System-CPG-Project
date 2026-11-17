%% make_result_figures.m - result figures of the IC sweep (two identical walkers), saved to ../results
%   fig_drift_vs_lag.png   lag drift rate vs lag: walking ~ -A sin(2 lag), running ~ +A sin(2 lag)
%   fig_gait_map.png       outcome of every start (initial lag x model)
%   fig_lag_vs_time.png    stride lag over time, three models side by side
%   fig_footfalls.png      footfall diagrams of the final gaits (pace, trot, running in step / half-step)
here = fileparts(mfilename('fullpath'));  res = fullfile(here, '..', 'results');
ld = @(f) load(fullfile(res, f));
sets = { 'walking, leg damper (30 s)',  {'ic_sweep_damper_a.mat', 'ic_sweep_damper_b.mat'};
         'walking, foot mass (30 s)',   {'ic_sweep_footmass_a.mat', 'ic_sweep_footmass_b.mat'};
         'running, leg damper (20 s)',  {'ic_sweep_stage3_run_damper.mat'} };
D = struct('name', {}, 'lag0', {}, 'lag_end', {}, 'drift', {}, 'fell', {}, 'locked', {}, 'cls', {}, 'G', {});
for s = 1:size(sets, 1)
    S = [];
    for f = 1:numel(sets{s,2}), L = ld(sets{s,2}{f});  S = [S, L.sweep.S]; end   %#ok<AGROW>
    d.name = sets{s,1};  n = numel(S);
    d.lag0 = zeros(n,1);  d.lag_end = nan(n,1);  d.drift = nan(n,1);  d.fell = false(n,1);  d.locked = false(n,1);  d.cls = cell(n,1);
    for i = 1:n
        G = S(i).G;  st = S(i).stance0;
        d.lag0(i) = mod(S(i).lag0 + 180*(st(2) ~= st(1)), 360);
        G.locked = ~G.fell && abs(G.drift) < 0.3 && G.lag_std < 5;
        [~, d.cls{i}] = sp_gait_label(G);
        d.lag_end(i) = G.lag_end;  d.drift(i) = G.drift;  d.fell(i) = G.fell;  d.locked(i) = G.locked;
    end
    d.G = [S.G];
    D(s) = d;
end
blue = [0.12 0.47 0.71];  red = [0.84 0.15 0.16];  grey = [0.5 0.5 0.5];

%% 1. drift rate vs lag
f1 = figure('Position', [50 50 1000 380], 'Color', 'w');
for s = [1 3]
    subplot(1, 2, 1 + (s == 3)); hold on; grid on;
    m = ~D(s).fell & ~isnan(D(s).drift);
    x = D(s).lag_end(m);  y = D(s).drift(m);
    A = -(sind(2*x)' * y) / (sind(2*x)' * sind(2*x));          % least squares fit  y = -A sin(2x)
    xx = 0:2:360;
    plot(xx, -A*sind(2*xx), '-', 'Color', grey, 'LineWidth', 1.5);
    plot(x, y, 'o', 'MarkerFaceColor', blue, 'Color', blue, 'MarkerSize', 5);
    for rr = [0 90 180 270 360], xline(rr, ':', 'Color', grey); end
    yline(0, 'k-');
    xlabel('stride lag, walker 2 vs 1 [deg]'); ylabel('lag drift rate [deg/s]');
    title(sprintf('%s:  d(lag)/dt \\approx %+.2f sin(2 lag) deg/s', D(s).name, -A), 'FontWeight', 'normal');
    xlim([0 360]);
    if s == 1
        text(0, min(y)*1.15, {'stable: 0 / 180 (in step)', 'unstable: 90 / 270 (half step)'}, 'FontSize', 9, 'VerticalAlignment', 'bottom');
    else
        text(0, min(y)*1.15, {'stable: 90 / 270 (half step)', 'unstable: 0 / 180 (in step)'}, 'FontSize', 9, 'VerticalAlignment', 'bottom');
    end
end
sgtitle('Phase dynamics of two identical walkers coupled by the object (each dot = one start of the sweep)');
print(f1, '-dpng', '-r140', fullfile(res, 'fig_drift_vs_lag.png'));

%% 2. gait map: outcome of every start
f2 = figure('Position', [50 50 1000 330], 'Color', 'w'); hold on; grid on;
classes = {'locked: pace', 'locked: trot', 'drifting to in-step (pace/trot)', 'locked: half-step', 'drifting to half-step', 'fell'};
cc = [0.13 0.55 0.13; 0.13 0.55 0.13; 0.55 0.80 0.55; 0.85 0.10 0.10; 0.95 0.55 0.55; 0.2 0.2 0.2];
mk = {'o', 's', 'o', 'o', 'o', 'x'};
h = gobjects(1, numel(classes));  seen = false(1, numel(classes));
for s = 1:3
    for k = 1:numel(classes)
        m = strcmp(D(s).cls, classes{k});
        if ~any(m), continue; end
        h(k) = plot(D(s).lag0(m), s*ones(sum(m),1) + 0.0, mk{k}, 'MarkerSize', 9, 'Color', cc(k,:), ...
                    'MarkerFaceColor', cc(k,:), 'LineWidth', 1.5, 'DisplayName', classes{k});
        seen(k) = true;
    end
end
set(gca, 'YTick', 1:3, 'YTickLabel', {D.name}, 'YLim', [0.4 3.6], 'XTick', 0:45:360);
for rr = [0 90 180 270 360], xline(rr, ':', 'Color', grey); end
xlabel('initial stride lag of walker 2 vs walker 1 [deg]  (0/180 = in step, 90/270 = half a step apart)');
title('Where each start ends up (pace and trot are the same body motion in 2-D; only the leg labels differ)', 'FontWeight', 'normal');
legend(h(seen), 'Location', 'eastoutside', 'FontSize', 8);
print(f2, '-dpng', '-r140', fullfile(res, 'fig_gait_map.png'));

%% 3. lag vs time, three models
f3 = figure('Position', [50 50 1300 380], 'Color', 'w');
for s = 1:3
    subplot(1, 3, s); hold on; grid on;
    G = D(s).G;  cm = lines(numel(G));
    for i = 1:numel(G)
        if isempty(G(i).tl), continue; end
        plot(G(i).tl, G(i).lag, '.-', 'Color', cm(i,:), 'LineWidth', 0.8);
        if G(i).fell, plot(G(i).t_end, G(i).lag(end), 'kx', 'MarkerSize', 9, 'LineWidth', 1.5); end
    end
    for rr = [0 90 180 270 360], yline(rr, ':', 'Color', grey); end
    ylim([-10 370]); xlabel('time [s]'); if s == 1, ylabel('stride lag W2 vs W1 [deg]'); end
    title(D(s).name, 'FontWeight', 'normal');
end
sgtitle('Stride lag over time for every start of the sweep (x = pair fell)');
print(f3, '-dpng', '-r140', fullfile(res, 'fig_lag_vs_time.png'));

%% 4. footfall diagrams of the final gaits
cf = fullfile(res, 'ic_cases.mat');
if exist(cf, 'file')
    load(cf, 'C');
    get = @(nm) C(strcmp({C.name}, nm));
    cases = {'walk_damper_lag60',    'walking, started 60 deg apart -> pace (0 deg)';
             'walk_damper_lag60_tr', 'same start, walker 2 on its other leg -> trot (180 deg): same body motion';
             'run_lag0',             'running, started in step -> stays in step';
             'run_lag30',            'running, started 30 deg apart -> half a step apart (90 deg)'};
    f4 = figure('Position', [50 50 1000 640], 'Color', 'w');
    legc = cat(3, [0.13 0.55 0.13; 0.85 0.10 0.10], [0.10 0.40 0.90; 0.55 0.20 0.70]);  legc = permute(legc, [1 3 2]);
    lab = {'W1 right', 'W1 left', 'W2 right', 'W2 left'};
    for q = 1:4
        c = get(cases{q,1});  R = c.R;
        ax = subplot(4, 1, q); hold on; box on;
        t1 = R.t(end) - 3;  m = R.t >= t1;  t = R.t(m);  L = R.legs(m, 1:4);
        for n = 1:4
            row = 5 - n;
            on = L(:, n) > 0.5;
            d = diff([0; on; 0]);  s0 = find(d == 1);  s1 = find(d == -1) - 1;
            for k = 1:numel(s0)
                patch([t(s0(k)) t(s1(k)) t(s1(k)) t(s0(k))] - t1, row + 0.35*[-1 -1 1 1], squeeze(legc(ceil(n/2), 2 - mod(n,2), :)).', 'EdgeColor', 'none');
            end
        end
        set(ax, 'YTick', 1:4, 'YTickLabel', fliplr(lab), 'YLim', [0.4 4.6], 'XLim', [0 3]);
        title(sprintf('%s   (final lag %.0f deg)', cases{q,2}, c.G.lag_end), 'FontWeight', 'normal', 'FontSize', 10);
        if q == 4, xlabel('time [s] (last 3 s of the run)'); end
    end
    sgtitle('Footfall diagrams of the gaits the model settles into (bar = foot on the ground)');
    print(f4, '-dpng', '-r140', fullfile(res, 'fig_footfalls.png'));
end
fprintf('figures written to %s\n', res);
