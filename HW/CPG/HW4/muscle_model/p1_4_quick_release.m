%% Problem 1.4 - Quick-release experiment with m = 400 kg
% Muscle locked at lopt with S = 1 from t = 0; released at tq = 1 s against
% a hanging mass. Plots lm(t) and Fm(t).  Uses run_quick_release.m + build_CE.m.
% Run as a script from the muscle_model folder.

clear; close all; clc;

%% Parameters (read by the Simulink blocks from the base workspace)
mp.Fmax = 6000;  mp.lopt = 0.08;  mp.vmax = 12 * mp.lopt;
mp.w = 0.56;  mp.c = 0.05;  mp.N = 1.5;  mp.K = 5;
mp.tauR = 0.030;  mp.tauF = 0.080;  mp.A0 = 0;

m    = 400;   % [kg] hanging mass
tq   = 1;     % [s]  release time
Tsim = 2;     % [s]

%% Simulate
[t, lm, Fm, vm, A] = run_quick_release(m, tq, Tsim);

%% Plot: full experiment (left) and zoom at release (right)
figure('Name', 'Quick release', 'Position', [100 100 900 600]);
win = {[0 Tsim], [tq-0.02 tq+0.2]};
for k = 1:2
    subplot(2,2,k);
    plot(t, 100*lm, 'LineWidth', 2); hold on; grid on;
    yline(100*mp.lopt, ':k'); xline(tq, '--k');
    xlim(win{k}); ylabel('l_m [cm]');
    if k == 1, title('Full experiment'); else, title('Zoom at release'); ylim([7 8.1]); end

    subplot(2,2,k+2);
    plot(t, Fm, 'r', 'LineWidth', 2); hold on; grid on;
    yline(m*9.81, ':k', 'mg'); xline(tq, '--k');
    xlim(win{k}); xlabel('time [s]'); ylabel('F_m [N]');
    if k == 2, ylim([3700 6100]); end
end

%% Key numbers
after = t >= tq;
[vmin, i] = min(vm);
fprintf('Force before release        : %.0f N (Fmax = %.0f N)\n', Fm(find(t < tq, 1, 'last')), mp.Fmax);
fprintf('Weight mg                   : %.0f N\n', m*9.81);
fprintf('Peak shortening velocity    : %.3f m/s (%.2f vmax) at %.0f ms after release\n', ...
        -vmin, -vmin/mp.vmax, 1000*(t(i)-tq));
fprintf('Final length                : %.2f cm = %.2f lopt\n', 100*lm(end), lm(end)/mp.lopt);
fprintf('Time for F to reach mg+5%%  : %.0f ms\n', 1000*(t(find(after & Fm < 1.05*m*9.81, 1)) - tq));

%% Export figure for the Overleaf write-up
if ~exist('figures', 'dir'), mkdir('figures'); end
exportgraphics(gcf, 'figures/p1_4_qr.png', 'Resolution', 300);
