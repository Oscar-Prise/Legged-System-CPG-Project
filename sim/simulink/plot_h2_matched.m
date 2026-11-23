function plot_h2_matched(matfile)
% PLOT_H2_MATCHED  Figure for H2 part A (matched clocks): final vs initial lag per stance combination.
load(matfile, 'A', 'h2_stance', 'h2_tEndA');
res = fileparts(matfile);  grey = [.5 .5 .5];
% figure: final vs initial lag per stance combination, and lag over time
fA = figure('Position', [50 50 1100 420], 'Color', 'w');
cols = lines(numel(h2_stance));
subplot(1, 2, 1); hold on; grid on;
for is = 1:numel(h2_stance)
    m = arrayfun(@(a) isequal(a.stance0, h2_stance{is}), A);
    x = [A(m).lag0_eff];  y = arrayfun(@(a) a.G.lag_end, A(m));  f = arrayfun(@(a) a.G.fell, A(m));
    plot(x(~f), y(~f), 'o', 'Color', cols(is,:), 'MarkerFaceColor', cols(is,:), 'DisplayName', sprintf('stance [%d %d]', h2_stance{is}));
    if any(f), plot(x(f), y(f), 'x', 'Color', cols(is,:), 'MarkerSize', 10, 'HandleVisibility', 'off'); end
end
plot([0 360], [0 360], '--', 'Color', [.75 .75 .75], 'HandleVisibility', 'off');
for r = [0 180 360], yline(r, ':', 'Color', grey, 'HandleVisibility', 'off'); end
yline(225, ':', 'Color', [.8 .3 .3], 'HandleVisibility', 'off');  text(5, 232, 'diagonal sequence', 'Color', [.8 .3 .3], 'FontSize', 8);
text(5, 188, 'trot', 'FontSize', 8);  text(5, 8, 'pace', 'FontSize', 8);
xlabel('initial stride lag, walker 2 vs 1 [deg]'); ylabel(sprintf('lag after %g s [deg]', h2_tEndA));
title('A: matched clocks, final vs initial lag', 'FontWeight', 'normal'); legend('Location', 'northwest', 'FontSize', 8);
xlim([-5 365]); ylim([-5 365]);
subplot(1, 2, 2); hold on; grid on;
for i = 1:numel(A)
    is = find(cellfun(@(s) isequal(s, A(i).stance0), h2_stance));
    plot(A(i).G.tl, A(i).G.lag, '.-', 'Color', cols(is,:), 'LineWidth', 0.8);
end
for r = [0 90 180 270 360], yline(r, ':', 'Color', grey); end
xlabel('time [s]'); ylabel('stride lag [deg]'); ylim([-10 370]);
title('A: lag over time, every start (colour = stance legs)', 'FontWeight', 'normal');
sgtitle('H2 part A: identical walkers with matched CPG clocks lock in step (pace / trot = same motion, legs relabelled)');
print(fA, '-dpng', '-r140', fullfile(res, 'fig_h2_matched.png'));
end
