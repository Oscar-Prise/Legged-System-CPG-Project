%% Problem 1.6 - Quick release with series elasticity (CE + SE)
% (a) m = 400 kg: compare lm(t), Fm(t) with and without SE.
% (b) Mass sweep with SE: m vs. initial slope of lm, and of lCE, vs. fv.
% Uses run_quick_release.m (CE only) and run_quick_release_SE.m (CE + SE).
% Run from the muscle_model folder.

clear; close all; clc;

%% Parameters (read by the Simulink blocks from the base workspace)
mp.Fmax = 6000;  mp.lopt = 0.08;  mp.vmax = 12 * mp.lopt;
mp.w = 0.56;  mp.c = 0.05;  mp.N = 1.5;  mp.K = 5;
mp.tauR = 0.030;  mp.tauF = 0.080;
mp.lslack = 0.23;   % [m] SE slack length (lecture VAS value)
mp.eref   = 0.04;   % [ ] SE reference strain at Fmax (lecture value)
mp.preA   = 0.01;   % [ ] pre-activation, avoids 0/0 in the inverted fv (as in m01)
mp.lce0   = mp.lopt;

g = 9.81;  tq = 1;  Tsim = 2;

%% (a) m = 400 kg with and without SE
m = 400;
mp.A0 = 0;        [t1, lm1, F1, v1]       = run_quick_release(m, tq, Tsim);
mp.A0 = mp.preA;  [t2, lm2, F2, v2, lce2] = run_quick_release_SE(m, tq, Tsim);

figure('Name', 'Quick release with SE', 'Position', [100 100 900 600]);
win = {[0 Tsim], [tq-0.02 tq+0.3]};
for k = 1:2
    subplot(2,2,k); hold on; grid on;
    plot(t1, 100*(lm1 - mp.lopt), 'LineWidth', 2);
    plot(t2, 100*(lm2 - mp.lopt - mp.lslack), '--', 'LineWidth', 2);
    xline(tq, '--k'); xlim(win{k}); ylabel('\Delta l_m [cm]');
    if k == 1, title('Full experiment'); legend({'CE only', 'CE + SE'}, 'Location', 'southwest');
    else, title('Zoom at release'); ylim([-1.5 0.15]); end
    subplot(2,2,k+2); hold on; grid on;
    plot(t1, F1, 'LineWidth', 2); plot(t2, F2, '--', 'LineWidth', 2);
    yline(m*g, ':k', 'mg'); xline(tq, '--k'); xlim(win{k}); xlabel('time [s]'); ylabel('F_m [N]');
    if k == 2, ylim([3300 6100]); end
end
if ~exist('figures', 'dir'), mkdir('figures'); end
exportgraphics(gcf, 'figures/p1_6_qr.png', 'Resolution', 300);

i0 = find(t2 < tq, 1, 'last');
fprintf('With SE, before release: Fm = %.0f N, lCE = %.3f lopt, lSE stretch = %.1f mm\n', ...
        F2(i0), lce2(i0)/mp.lopt, 1000*(lm2(i0) - lce2(i0) - mp.lslack));
fprintf('With SE, after release : min Fm = %.0f N (undershoot below mg = %.0f N), peak v_m = %.3f m/s\n', ...
        min(F2(t2 > tq)), m*g, min(v2));
fprintf('CE only, after release : peak v_m = %.3f m/s\n', min(v1));

%% (b) Mass sweep with SE: slope of lm and slope of lCE
masses = [1 10 50 100 200 400 600 700 800];
twin = 0.15;  nM = numel(masses);
slope_noSE = zeros(1,nM);  slope_lm = zeros(1,nM);  slope_lce = zeros(1,nM);

for k = 1:nM
    m = masses(k);  lifts = (m*g <= mp.Fmax);
    mp.A0 = 0;        [t, ~, ~, vm] = run_quick_release(m, tq, 1.35);
    win = (t > tq + 0.002) & (t < tq + twin);
    slope_noSE(k) = local_pick(vm, gradient(vm, t), t, win, lifts);

    mp.A0 = mp.preA;  [t, ~, ~, vm, lce] = run_quick_release_SE(m, tq, 1.35);
    win = (t > tq + 0.002) & (t < tq + twin);
    vce = gradient(lce, t);
    slope_lm(k)  = local_pick(vm,  gradient(vm, t),  t, win, lifts);
    slope_lce(k) = local_pick(vce, gradient(vce, t), t, win, lifts);
end

v_curve  = linspace(-mp.vmax, 0.15, 500);
fv_curve = arrayfun(@(v) CE_force_velocity_relation(v / mp.vmax, mp), v_curve);

figure('Name', 'Mass vs slope with SE', 'Position', [100 100 640 460]); hold on; grid on;
plot(v_curve, mp.Fmax * fv_curve / g, 'k', 'LineWidth', 1.5, 'DisplayName', 'm = F_{max} f_v(v) / g');
plot(slope_noSE, masses, 'o', 'MarkerSize', 7, 'MarkerFaceColor', [.85 .2 .2], 'MarkerEdgeColor', 'k', 'DisplayName', 'CE only: slope of l_m');
plot(slope_lm,   masses, 's', 'MarkerSize', 7, 'MarkerFaceColor', [.2 .6 .2], 'MarkerEdgeColor', 'k', 'DisplayName', 'CE + SE: slope of l_m');
plot(slope_lce,  masses, '^', 'MarkerSize', 7, 'MarkerFaceColor', [1 .5 .1],  'MarkerEdgeColor', 'k', 'DisplayName', 'CE + SE: dl_{CE}/dt only');
xline(0, ':', 'Color', [.5 .5 .5], 'HandleVisibility', 'off');
yline(mp.Fmax/g, '--', 'F_{max}/g', 'Color', [.5 .5 .5], 'HandleVisibility', 'off');
xlim([-1.02 0.12]); ylim([0 900]);
xlabel('slope right after t_q [m/s]  (<0 shortening)'); ylabel('m [kg]'); legend('Location', 'northwest');
exportgraphics(gcf, 'figures/p1_6_sweep.png', 'Resolution', 300);

fprintf('\n   m [kg]   CE only   SE: dl_m/dt   SE: dl_CE/dt   from fv\n');
for k = 1:nM
    v_pred = CE_inverse_force_velocity_relation(masses(k)*g/mp.Fmax, mp) * mp.vmax;
    fprintf('  %5d    %+.3f     %+.3f        %+.3f       %+.3f\n', masses(k), slope_noSE(k), slope_lm(k), slope_lce(k), v_pred);
end

%% helper: value of v "right after release" (extremum if the muscle lifts the mass, plateau otherwise)
function s = local_pick(v, a, t, win, lifts)
if lifts
    [~, i] = max(abs(v .* win));
else
    a(~win) = inf;  [~, i] = min(abs(a));
end
s = v(i);
end
