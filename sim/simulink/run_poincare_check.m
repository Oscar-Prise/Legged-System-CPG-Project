%% run_poincare_check.m - Poincare-map stability of the paired gaits (two identical walkers)
%
% (1) Full return map of the pair at walker 1's apex (sp_pair_poincare): Jacobian eigenvalues at
%     walking in-step, walking half-step, running half-step, running in-step.
% (2) 1-D stroboscopic return map of the stride lag from the IC sweep data:  lag_{n+1} vs lag_n,
%     fixed points at the diagonal crossings, slope = eigenvalue of the lag mode.
% Saves ../results/poincare_pairs.mat, fig_poincare_eigs.png, fig_poincare_lag_map.png

here = fileparts(mfilename('fullpath'));  res = fullfile(here, '..', 'results');
cases = { % name                        preset         lag0  dl    tRun
          'walking, in step (lag 0)',    'stage3_pair',   0,  [],   24;
          'walking, half step (lag 90)', 'stage3_pair',  90,  [],   24;
          'running, half step (lag 90)', 'stage3_run',   90,  0.04, 20;
          'running, in step (lag 0)',    'stage3_run',    0,  0.04, 20 };
Pc = cell(size(cases, 1), 1);
for n = 1:size(cases, 1)
    fprintf('\n=== %s ===\n', cases{n, 1});
    preset = cases{n, 2};  legmodel = 'damper';  pairtype = 'identical';
    lag0 = cases{n, 3};  stance0 = [1 1];
    if isempty(cases{n, 4}), clear dl_set; else, dl_set = cases{n, 4}; end
    cpg_pair_params;
    R = sp_fixedstep_check(cases{n, 5}, mp, ic);
    G = sp_gait_class(R, mp, cases{n, 5});
    fprintf('run: %s, lag_end %.1f deg\n', G.label, G.lag_end);
    P = sp_pair_poincare(mp, R);
    P.name = cases{n, 1};  P.lag_run = G.lag_end;
    % which coordinates the leading eigenvector lives on
    v = abs(P.V(:, 1));  [~, o] = sort(v, 'descend');
    P.lead = sprintf('%s %.2f, %s %.2f, %s %.2f', P.names{o(1)}, v(o(1)), P.names{o(2)}, v(o(2)), P.names{o(3)}, v(o(3)));
    fprintf('leading |eig| = %.4f (%s)\n', abs(P.lam(1)), P.lead);
    Pc{n} = P;
end
save(fullfile(res, 'poincare_pairs.mat'), 'Pc');

%% eigenvalue figure
f1 = figure('Position', [50 50 1100 320], 'Color', 'w');
th = linspace(0, 2*pi, 200);
for n = 1:4
    P = Pc{n};
    subplot(1, 4, n); hold on; axis equal; grid on;
    plot(cos(th), sin(th), '-', 'Color', [.6 .6 .6]);
    plot(real(P.lam), imag(P.lam), 'o', 'MarkerFaceColor', [0.12 0.47 0.71], 'Color', [0.12 0.47 0.71], 'MarkerSize', 6);
    plot(real(P.lam(1)), imag(P.lam(1)), 'o', 'Color', [0.84 0.15 0.16], 'MarkerSize', 11, 'LineWidth', 1.5);
    xlim([-1.3 1.3]); ylim([-1.3 1.3]);
    if max(abs(P.lam)) < 1, v = 'STABLE'; else, v = 'UNSTABLE'; end
    title(sprintf('%s\nmax |\\lambda| = %.3f  (%s)', P.name, max(abs(P.lam)), v), 'FontWeight', 'normal', 'FontSize', 9);
    xlabel('Re \lambda'); if n == 1, ylabel('Im \lambda'); end
end
sgtitle('Return map of the coupled pair at walker 1''s apex: eigenvalues of the numerical Jacobian (red ring = largest)');
print(f1, '-dpng', '-r140', fullfile(res, 'fig_poincare_eigs.png'));

%% 1-D lag return map from the sweep data
sets = { 'walking, leg damper',  {'ic_sweep_damper_a.mat', 'ic_sweep_damper_b.mat', 'ic_sweep_damper_long.mat'};
         'walking, foot mass',   {'ic_sweep_footmass_a.mat', 'ic_sweep_footmass_b.mat'};
         'running, leg damper',  {'ic_sweep_stage3_run_damper.mat'} };
f2 = figure('Position', [50 50 1300 620], 'Color', 'w');
lagmap = struct('name', {}, 'A', {}, 'T', {}, 'lam_in', {}, 'lam_half', {});
for s = 1:3
    S = [];
    for f = 1:numel(sets{s, 2}), L = load(fullfile(res, sets{s, 2}{f}));  S = [S, L.sweep.S]; end   %#ok<AGROW>
    x = [];  y = [];  Tm = [];
    for i = 1:numel(S)
        G = S(i).G;  l = G.lag(:);  t = G.tl(:);
        if G.fell, l = l(t < G.t_end - 3); t = t(t < G.t_end - 3); end
        if numel(l) < 3, continue; end
        x = [x; l(1:end-1)];  y = [y; l(2:end)];  Tm = [Tm; diff(t)];   %#ok<AGROW>
    end
    d = mod(y - x + 180, 360) - 180;                        % lag change per stride, wrapped
    A = -(sind(2*x)' * d) / (sind(2*x)' * sind(2*x));         % fit  d = -A sin(2 lag)  [deg/stride]
    T = median(Tm);
    lam_in = 1 - 2*A*pi/180;  lam_half = 1 + 2*A*pi/180;      % slopes of lag_{n+1}(lag_n) at 0/180 and 90/270
    lagmap(s).name = sets{s, 1};  lagmap(s).A = A;  lagmap(s).T = T;  lagmap(s).lam_in = lam_in;  lagmap(s).lam_half = lam_half;
    subplot(2, 3, s); hold on; grid on;
    plot([0 360], [0 360], '-', 'Color', [.6 .6 .6]);
    plot(x, y, '.', 'Color', [0.12 0.47 0.71], 'MarkerSize', 5);
    xx = 0:2:360;  plot(xx, mod(xx - A*sind(2*xx), 360), '-', 'Color', [0.84 0.15 0.16], 'LineWidth', 1.2);
    xlim([0 360]); ylim([0 360]); set(gca, 'XTick', 0:90:360, 'YTick', 0:90:360);
    xlabel('lag_n [deg]'); if s == 1, ylabel('lag_{n+1} [deg]'); end
    title(sprintf('%s: lag return map', sets{s, 1}), 'FontWeight', 'normal');
    subplot(2, 3, 3 + s); hold on; grid on;
    plot(x, d, '.', 'Color', [0.12 0.47 0.71], 'MarkerSize', 5);
    plot(xx, -A*sind(2*xx), '-', 'Color', [0.84 0.15 0.16], 'LineWidth', 1.2);
    yline(0, 'k-');  for rr = [0 90 180 270 360], xline(rr, ':', 'Color', [.5 .5 .5]); end
    xlim([0 360]); set(gca, 'XTick', 0:90:360);
    xlabel('lag_n [deg]'); if s == 1, ylabel('lag_{n+1} - lag_n [deg/stride]'); end
    title(sprintf('fit %+.2f sin(2 lag):  slope at 0/180 = %.3f, at 90/270 = %.3f', -A, lam_in, lam_half), 'FontWeight', 'normal', 'FontSize', 9);
end
sgtitle('Stroboscopic return map of the stride lag (section: walker 1 right-foot touchdown); fixed points where the map meets the diagonal, stable where the slope is < 1');
print(f2, '-dpng', '-r140', fullfile(res, 'fig_poincare_lag_map.png'));
save(fullfile(res, 'poincare_pairs.mat'), 'Pc', 'lagmap');

fprintf('\n%-28s %-10s %-9s %-8s %s\n', 'gait', 'max|eig|', 'resid', 'T [s]', 'leading eigenvector on');
for n = 1:4, P = Pc{n}; fprintf('%-28s %-10.4f %-9.1e %-8.4f %s\n', P.name, max(abs(P.lam)), P.resid, P.T, P.lead); end
fprintf('\n1-D lag map slopes (eigenvalue of the lag mode per stride):\n');
for s = 1:3, fprintf('  %-22s at in-step %.4f, at half-step %.4f  (T = %.3f s)\n', lagmap(s).name, lagmap(s).lam_in, lagmap(s).lam_half, lagmap(s).T); end
