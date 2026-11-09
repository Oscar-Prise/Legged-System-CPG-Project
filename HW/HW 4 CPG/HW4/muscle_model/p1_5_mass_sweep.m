%% Problem 1.5 - Quick release for several masses: slope vs. force-velocity relation
% Repeats the experiment of 1.4 for m = 1 ... 800 kg, measures the slope of
% lm(t) right after release (the extremum / plateau of vm), and compares
% m-vs-slope with the force-velocity relation used in the model.
% Uses run_quick_release.m + build_CE.m.  Run from the muscle_model folder.

clear; close all; clc;

%% Parameters (read by the Simulink blocks from the base workspace)
mp.Fmax = 6000;  mp.lopt = 0.08;  mp.vmax = 12 * mp.lopt;
mp.w = 0.56;  mp.c = 0.05;  mp.N = 1.5;  mp.K = 5;
mp.tauR = 0.030;  mp.tauF = 0.080;  mp.A0 = 0;

g      = 9.81;
masses = [1 10 50 100 200 400 600 700 800];   % [kg]
tq     = 1;                                   % [s]
Tsim   = 1.3;                                 % [s]  only need ~0.3 s after release
twin   = 0.15;                                % [s]  window after release to look for the slope

%% Run the sweep
nM    = numel(masses);
slope = zeros(1, nM);  tslope = zeros(1, nM);  res = cell(1, nM);
cols  = parula(nM + 1);

figure('Name', 'Velocity after release', 'Position', [100 100 1000 420]);
subplot(1,2,1); hold on; grid on;
for k = 1:nM
    m = masses(k);
    [t, lm, Fm, vm] = run_quick_release(m, tq, Tsim);
    res{k} = struct('t', t, 'lm', lm, 'Fm', Fm, 'vm', vm);

    % slope of lm right after tq = velocity right after the initial transient
    win = (t > tq + 0.002) & (t < tq + twin);
    if m * g <= mp.Fmax            % muscle lifts the mass: take the velocity extremum
        [~, i] = max(abs(vm .* win));
    else                           % mass wins: take the plateau (smallest |acceleration|)
        acc = gradient(vm, t);  acc(~win) = inf;
        [~, i] = min(abs(acc));
    end
    slope(k) = vm(i);  tslope(k) = t(i) - tq;

    plot(1000*(t - tq), vm, 'Color', cols(k,:), 'LineWidth', 1.8, 'DisplayName', sprintf('%g kg', m));
    plot(1000*tslope(k), slope(k), 'o', 'MarkerFaceColor', cols(k,:), 'MarkerEdgeColor', 'k', 'HandleVisibility', 'off');
end
yline(0, 'k', 'HandleVisibility', 'off');
xlim([-10 1000*twin]); ylim([-1 0.12]);
xlabel('time after release [ms]'); ylabel('v_m = dl_m/dt [m/s]');
title('Muscle velocity after release'); legend('Location', 'southeast', 'NumColumns', 2);

%% Mass vs. slope, compared with the force-velocity relation
% Right after release, at l = lopt and A = 1:  m g = Fmax * fv(v)  ->  m(v) = Fmax fv(v) / g
v_curve = linspace(-mp.vmax, 0.15, 500);
fv_curve = arrayfun(@(v) CE_force_velocity_relation(v / mp.vmax, mp), v_curve);
m_curve  = mp.Fmax * fv_curve / g;

subplot(1,2,2); hold on; grid on;
plot(v_curve, m_curve, 'k', 'LineWidth', 1.5, 'DisplayName', 'm = F_{max} f_v(v) / g');
plot(slope, masses, 'o', 'MarkerSize', 7, 'MarkerFaceColor', [0.85 0.2 0.2], 'MarkerEdgeColor', 'k', ...
     'DisplayName', 'simulation: m vs. slope');
xline(0, ':', 'Color', [.5 .5 .5], 'HandleVisibility', 'off');
yline(mp.Fmax/g, '--', 'F_{max}/g', 'Color', [.5 .5 .5], 'HandleVisibility', 'off');
xlim([-1 0.1]); ylim([0 900]);
xlabel('slope dl_m/dt right after t_q [m/s]  (<0 shortening)'); ylabel('m [kg]');
title('Mass vs. initial slope'); legend('Location', 'northwest');

%% Table: measured slope vs. velocity predicted from fv alone
fprintf('   m [kg]   slope [m/s]   v/vmax   t [ms]   v predicted by fv [m/s]\n');
for k = 1:nM
    fv_t = masses(k) * g / mp.Fmax;                       % required fv at lopt, A = 1
    v_pred = CE_inverse_force_velocity_relation(fv_t, mp) * mp.vmax;
    fprintf('  %5d     %+.4f     %+.3f    %5.1f      %+.4f\n', masses(k), slope(k), slope(k)/mp.vmax, 1000*tslope(k), v_pred);
end

%% Export figure for the Overleaf write-up
if ~exist('figures', 'dir'), mkdir('figures'); end
exportgraphics(gcf, 'figures/p1_5_sweep.png', 'Resolution', 300);
