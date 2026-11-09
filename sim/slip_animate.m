function slip_animate(out, r, speed)
% SLIP_ANIMATE  Stick-figure animation of one or two walkers (+ object).
%   slip_animate(out, r)        out from slip_sim, r from slip_post
%   slip_animate(out, r, 0.5)   half speed

if nargin < 3, speed = 1; end
P = out.P;  nW = numel(out.W);  fps = 30;
% contact/foot data per sample, from the segment each sample belongs to
segEnd = cellfun(@(s) s.t(end), out.seg);
tf = (r.t(1):speed/fps:r.t(end)).';
figure('Name', 'animation', 'Color', 'w');
col = lines(2);
for n = 1:numel(tf)
    [~, k] = min(abs(r.t - tf(n)));
    s = find(segEnd >= r.t(k), 1);  Wk = out.seg{s}.W;
    cla; hold on; axis equal;
    xm = mean(r.x(k, :));
    plot([xm - 3, xm + 3], [0 0], 'k', 'LineWidth', 1);
    for i = 1:nW
        x = r.x(k, i);  z = r.z(k, i);
        for j = 1:2
            if Wk(i).contact(j)
                plot([Wk(i).foot(j) x], [0 z], '-', 'Color', col(i, :), 'LineWidth', 2 + (j == 1));
            else                                      % swing leg at the angle of attack
                plot([x x + P.l0*cos(P.a0)], [z z - P.l0*sin(P.a0)], ':', 'Color', col(i, :), 'LineWidth', 1.5);
            end
        end
        plot(x, z, 'o', 'MarkerSize', 14, 'MarkerFaceColor', col(i, :), 'MarkerEdgeColor', 'k');
    end
    if out.opt.coupled
        xo = r.y(k, 4*nW + 1);  hb = 0.9;
        plot([r.x(k, 1) xo], [hb hb], 'k-');  plot([xo r.x(k, 2)], [hb hb], 'k-');
        plot(xo, hb, 'ks', 'MarkerSize', 10, 'MarkerFaceColor', [.6 .6 .6]);
    end
    xlim([xm - 2, xm + 2]);  ylim([-0.1 1.3]);
    title(sprintf('t = %.2f s', r.t(k)));
    drawnow;
end
end
