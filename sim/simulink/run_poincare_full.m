%% run_poincare_full.m - Poincare-map stability with the FULL CPG model (foot-mass legs, CPG-driven
% swing, touchdown reset, load feedback sigma = 3), plus the leg-damper model with sigma = 3 for
% comparison.  Saves ../results/poincare_full.mat and fig_poincare_eigs_full.png
here = fileparts(mfilename('fullpath'));  res = fullfile(here, '..', 'results');
cases = { % name                                      legmodel    lag0  sigma tRun
          'foot mass, sigma 3: in step (lag 0)',       'footmass',   0,   3,  30;
          'foot mass, sigma 3: half step (lag 90)',    'footmass',  90,   3,  10;
          'foot mass, sigma 0: in step (lag 0)',       'footmass',   0,   0,  30;
          'leg damper, sigma 3: in step (lag 0)',      'damper',     0,   3,  24 };
Pc = cell(size(cases, 1), 1);
for n = 1:size(cases, 1)
    fprintf('\n=== %s ===\n', cases{n, 1});
    preset = 'stage3_pair';  legmodel = cases{n, 2};  pairtype = 'identical';
    lag0 = cases{n, 3};  stance0 = [1 1];  sigma_set = cases{n, 4};  clear dl_set
    cpg_pair_params;
    R = sp_fixedstep_check(cases{n, 5}, mp, ic);
    G = sp_gait_class(R, mp, cases{n, 5}, min(10, cases{n, 5} - 2));
    fprintf('run: %s, lag_end %.1f deg, stop %d at %.1f s\n', G.label, G.lag_end, R.stop, R.t_end);
    try
        P = sp_pair_poincare(mp, R);
        v = abs(P.V(:, 1));  [~, o] = sort(v, 'descend');
        P.lead = sprintf('%s %.2f, %s %.2f, %s %.2f', P.names{o(1)}, v(o(1)), P.names{o(2)}, v(o(2)), P.names{o(3)}, v(o(3)));
        fprintf('leading |eig| = %.4f (%s)\n', abs(P.lam(1)), P.lead);
    catch ME
        fprintf('Poincare map failed: %s\n', ME.message);
        P = struct('ok', false, 'lam', NaN, 'resid', NaN, 'T', NaN, 'lead', ME.message, 'names', {{}});
    end
    P.name = cases{n, 1};  P.lag_run = G.lag_end;
    Pc{n} = P;
end
save(fullfile(res, 'poincare_full.mat'), 'Pc');

f1 = figure('Position', [50 50 1100 320], 'Color', 'w');
th = linspace(0, 2*pi, 200);
for n = 1:4
    P = Pc{n};
    subplot(1, 4, n); hold on; axis equal; grid on;
    plot(cos(th), sin(th), '-', 'Color', [.6 .6 .6]);
    if isfield(P, 'ok') && P.ok
        plot(real(P.lam), imag(P.lam), 'o', 'MarkerFaceColor', [0.12 0.47 0.71], 'Color', [0.12 0.47 0.71], 'MarkerSize', 6);
        plot(real(P.lam(1)), imag(P.lam(1)), 'o', 'Color', [0.84 0.15 0.16], 'MarkerSize', 11, 'LineWidth', 1.5);
        if P.resid > 2e-3, v = sprintf('inconclusive, residual %.0e', P.resid); elseif max(abs(P.lam)) < 1, v = 'STABLE'; else, v = 'UNSTABLE'; end
        title(sprintf('%s\nmax |\\lambda| = %.3f  (%s)', P.name, max(abs(P.lam)), v), 'FontWeight', 'normal', 'FontSize', 9);
    else
        title(sprintf('%s\nno fixed point found', P.name), 'FontWeight', 'normal', 'FontSize', 9);
    end
    xlim([-1.3 1.3]); ylim([-1.3 1.3]); xlabel('Re \lambda'); if n == 1, ylabel('Im \lambda'); end
end
sgtitle('Return map at walker 1''s apex, full CPG model (foot-mass legs, CPG swing, load feedback)');
print(f1, '-dpng', '-r140', fullfile(res, 'fig_poincare_eigs_full.png'));
fprintf('\n%-42s %-10s %-9s %-8s %s\n', 'gait', 'max|eig|', 'resid', 'T [s]', 'leading eigenvector on');
for n = 1:4, P = Pc{n}; fprintf('%-42s %-10.4f %-9.1e %-8.4f %s\n', P.name, max(abs(P.lam)), P.resid, P.T, P.lead); end
