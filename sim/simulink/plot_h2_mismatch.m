function plot_h2_mismatch(matfile)
% PLOT_H2_MISMATCH  Figure for H2 part B (clock mismatch): locked lag, slip rate and stride times vs omega2/omega1.
% Lags are shown wrapped to (-180, 180] so that in-step locks sit at 0 and one-step-offset locks at +-180.
load(matfile, 'B', 'h2_ratios', 'h2_cpg', 'h2_tEndB');
res = fileparts(matfile);  grey = [.5 .5 .5];
wrap = @(x) mod(x + 180, 360) - 180;
fB = figure('Position', [50 50 1300 420], 'Color', 'w');
cols = [0.12 0.47 0.71; 0.84 0.15 0.16];
for ic_ = 1:numel(h2_cpg)
    m = [B.reset] == h2_cpg{ic_}(1) & [B.sigma] == h2_cpg{ic_}(2);
    r = [B(m).ratio];  Gs = [B(m).G];  lag = wrap([Gs.lag_end]);  locked = [Gs.locked];  fell = [Gs.fell];
    nm = sprintf('reset %d, \\sigma = %g', h2_cpg{ic_}(1), h2_cpg{ic_}(2));
    subplot(1, 3, 1); hold on; grid on;
    plot(r(locked), lag(locked), 'o-', 'Color', cols(ic_,:), 'MarkerFaceColor', cols(ic_,:), 'DisplayName', [nm ' (locked)']);
    plot(r(~locked & ~fell), lag(~locked & ~fell), 's', 'Color', cols(ic_,:), 'DisplayName', [nm ' (slipping)']);
    if any(fell), plot(r(fell), lag(fell), 'x', 'Color', cols(ic_,:), 'MarkerSize', 10, 'DisplayName', [nm ' (fell)']); end
    subplot(1, 3, 2); hold on; grid on;
    plot(r, [Gs.drift], 'o-', 'Color', cols(ic_,:), 'MarkerFaceColor', cols(ic_,:), 'DisplayName', nm);
    subplot(1, 3, 3); hold on; grid on;
    plot(r, [B(m).T1], 'o-', 'Color', cols(ic_,:), 'MarkerFaceColor', cols(ic_,:), 'DisplayName', [nm ' walker 1']);
    plot(r, [B(m).T2], 's--', 'Color', cols(ic_,:), 'DisplayName', [nm ' walker 2']);
end
subplot(1, 3, 1);
for rr = [-180 0 180], yline(rr, ':', 'Color', grey, 'HandleVisibility', 'off'); end
yline(-135, ':', 'Color', [.8 .3 .3], 'HandleVisibility', 'off');  text(min(h2_ratios), -128, 'diagonal sequence 225 (= -135)', 'Color', [.8 .3 .3], 'FontSize', 8);
text(min(h2_ratios), 8, 'in step (pace)', 'FontSize', 8);  text(min(h2_ratios), 188, 'one step apart (trot label)', 'FontSize', 8);
xlabel('\omega_2 / \omega_1'); ylabel(sprintf('lag after %g s [deg], wrapped', h2_tEndB)); ylim([-190 190]);
title('B: locked lag vs clock mismatch', 'FontWeight', 'normal'); legend('Location', 'northwest', 'FontSize', 7);
subplot(1, 3, 2); yline(0, 'k-', 'HandleVisibility', 'off');
xlabel('\omega_2 / \omega_1'); ylabel('lag drift over the last 10 s [deg/s]');
title('B: slip rate (0 = entrained)', 'FontWeight', 'normal'); legend('Location', 'northwest', 'FontSize', 7);
subplot(1, 3, 3);
xlabel('\omega_2 / \omega_1'); ylabel('stride time, last 10 s [s]');
title('B: stride times of the two walkers (equal = one shared rhythm)', 'FontWeight', 'normal'); legend('Location', 'best', 'FontSize', 7);
sgtitle('H2 part B: identical walkers with mismatched CPG clocks, started in step');
print(fB, '-dpng', '-r140', fullfile(res, 'fig_h2_mismatch.png'));
end
