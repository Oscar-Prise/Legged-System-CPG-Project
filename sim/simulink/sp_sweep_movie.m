function sp_sweep_movie(C, out, windows, caption, ttl)
% SP_SWEEP_MOVIE  Montage video of several runs side by side: a 2 x 3 grid of simple side views
% (bodies, stance legs, object, arms) with the stride lag of each pair, and below them the lag of
% every run against time with a moving cursor.  Writes out.mp4 and out.gif.
%
%   sp_sweep_movie(C, '../results/sweep_walk', [0 6; 114 120], {'start', 'end'}, 'title')
%
% C = struct array (from make_result_cases.m) with fields R (thinned), mp, G, lag0.
n = numel(C);  nc = 3;  nr = ceil(n/nc);
if nargin < 3 || isempty(windows), windows = [C(1).R.t(1) C(1).R.t(end)]; end
if nargin < 4, caption = repmat({''}, 1, size(windows, 1)); end
if nargin < 5, ttl = ''; end
col  = [0.13 0.55 0.13; 0.85 0.10 0.10];                    % walker 1 green, walker 2 red
legc = cat(3, [0.13 0.55 0.13; 0.85 0.10 0.10], [0.10 0.40 0.90; 0.55 0.20 0.70]);  % (walker, leg, rgb)
legc = permute(legc, [1 3 2]);
cmap = lines(n);
dt = C(1).mp.dt_anim;

hFig = figure('Position', [40 40 1200 780], 'Color', 'w', 'Visible', 'on');
H = struct();
for q = 1:n
    r = ceil(q/nc);  c = q - nc*(r-1);
    ax = axes('Parent', hFig, 'Position', [0.04 + (c-1)*0.32, 0.72 - (r-1)*0.30, 0.29, 0.24]);
    hold(ax, 'on');  box(ax, 'on');  daspect(ax, [1 1 1]);  set(ax, 'FontSize', 8);
    plot(ax, [-10 1000], [0 0], 'k-', 'LineWidth', 1.2);
    mp = C(q).mp;
    for i = 1:2
        for j = 1:2
            H.leg(q,i,j)  = plot(ax, [0 0], [0 0], '-', 'Color', squeeze(legc(i,j,:)).', 'LineWidth', 2, 'Visible', 'off');
            H.foot(q,i,j) = plot(ax, 0, 0, 'o', 'MarkerSize', 4, 'MarkerFaceColor', squeeze(legc(i,j,:)).', 'MarkerEdgeColor', 'none', 'Visible', 'off');
        end
        H.body(q,i) = plot(ax, 0, 0, 'o', 'MarkerSize', 14, 'MarkerFaceColor', col(i,:), 'MarkerEdgeColor', 'k');
    end
    H.arm(q) = plot(ax, [0 0 0], [0 0 0], '-', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.2);
    H.obj(q) = plot(ax, 0, 0, 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k');
    H.txt(q) = text(0.02, 0.93, '', 'Parent', ax, 'Units', 'normalized', 'FontSize', 9, 'Color', cmap(q,:), 'FontWeight', 'bold');
    ylim(ax, [-0.1 1.5]);  xlim(ax, [-1 4]);
    title(ax, sprintf('start lag %g deg', C(q).lag0), 'FontSize', 9, 'FontWeight', 'normal', 'Color', cmap(q,:));
    if c == 1, ylabel(ax, 'z [m]'); end
    H.ax(q) = ax;
end
% lag-vs-time panel
axl = axes('Parent', hFig, 'Position', [0.07 0.06 0.90 0.17]);  hold(axl, 'on');  box(axl, 'on');  grid(axl, 'on');
set(axl, 'FontSize', 8);
for q = 1:n
    plot(axl, C(q).G.tl, C(q).G.lag, '.-', 'Color', cmap(q,:), 'LineWidth', 1);
end
for rr = [0 90 180 270 360], yline(axl, rr, ':', 'Color', [.5 .5 .5]); end
ylim(axl, [-10 370]);  xlim(axl, [0 C(1).R.t(end)]);
xlabel(axl, 'time [s]');  ylabel(axl, 'stride lag W2 vs W1 [deg]');
H.cursor = plot(axl, [0 0], [-10 370], 'k-', 'LineWidth', 1.5);
H.cap = annotation(hFig, 'textbox', [0.60 0.955 0.38 0.04], 'String', '', 'EdgeColor', 'none', ...
                   'HorizontalAlignment', 'right', 'FontSize', 11, 'FontWeight', 'bold');
annotation(hFig, 'textbox', [0.02 0.955 0.60 0.04], 'String', ttl, 'EdgeColor', 'none', 'FontSize', 12, 'FontWeight', 'bold');

vw = VideoWriter([out '.mp4'], 'MPEG-4');  vw.FrameRate = round(1/dt);  open(vw);
first = true;  nfr = 0;
for w = 1:size(windows, 1)
    for t = windows(w,1):dt:windows(w,2)
        for q = 1:n
            R = C(q).R;  mp = C(q).mp;
            k = find(R.t >= t - 1e-9, 1);  if isempty(k), k = numel(R.t); end
            pos = R.pos(k,:);  legs = R.legs(k,:);
            foot = mp.legmodel > 1.5;
            for i = 1:2
                x = pos(2*i-1);  z = pos(2*i);
                set(H.body(q,i), 'XData', x, 'YData', z);
                for j = 1:2
                    m = 2*(i-1) + j;
                    if legs(m) > 0.5
                        set(H.leg(q,i,j), 'XData', [x legs(4+m)], 'YData', [z 0], 'Visible', 'on', 'LineWidth', 2 + 2*(legs(12+m) > 0.05));
                        set(H.foot(q,i,j), 'Visible', 'off');
                    elseif foot
                        set(H.leg(q,i,j), 'XData', [x legs(16+m)], 'YData', [z legs(20+m)], 'Visible', 'on', 'LineWidth', 1);
                        set(H.foot(q,i,j), 'XData', legs(16+m), 'YData', legs(20+m), 'Visible', 'on');
                    else
                        set(H.leg(q,i,j), 'Visible', 'off');  set(H.foot(q,i,j), 'Visible', 'off');
                    end
                end
            end
            zo = 0.5*(pos(2) + pos(4));
            set(H.obj(q), 'XData', pos(5), 'YData', zo);
            set(H.arm(q), 'XData', [pos(1) pos(5) pos(3)], 'YData', [pos(2) zo pos(4)]);
            xc = 0.5*(pos(1) + pos(3));  set(H.ax(q), 'XLim', xc + [-2.5 2.5]);
            kl = find(C(q).G.tl <= t, 1, 'last');
            if ~isempty(kl), s = sprintf('lag %3.0f deg', C(q).G.lag(kl)); else, s = ''; end
            if C(q).G.fell && t >= C(q).G.t_end, s = sprintf('fell at %.1f s', C(q).G.t_end); end
            set(H.txt(q), 'String', sprintf('t = %.1f s   %s', R.t(k), s));
        end
        set(H.cursor, 'XData', [t t]);
        set(H.cap, 'String', caption{w});
        fr = getframe(hFig);  writeVideo(vw, fr);
        [Im, map] = rgb2ind(fr.cdata, 128);
        if first, imwrite(Im, map, [out '.gif'], 'gif', 'LoopCount', Inf, 'DelayTime', dt);  first = false;
        else, imwrite(Im, map, [out '.gif'], 'gif', 'WriteMode', 'append', 'DelayTime', dt); end
        nfr = nfr + 1;
    end
end
close(vw);  close(hFig);
fprintf('sp_sweep_movie: %d frames -> %s.mp4 / .gif\n', nfr, out);
end
