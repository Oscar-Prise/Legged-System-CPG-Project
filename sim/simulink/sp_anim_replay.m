function sp_anim_replay(R, mp, out, windows, caption)
% SP_ANIM_REPLAY  Replay a sp_fixedstep_check run through the sp_anim drawing code (no Simulink)
% and write out.mp4 and out.gif (25 fps, real time).
%
%   sp_anim_replay(R, mp, '../results/case', [0 6; 24 30], {'start', 'end'})
%
% windows = [t1 t2; ...] time windows to show one after the other (default: the whole run);
% caption = one label per window, shown top right.  R needs t, pos, legs, phi.
if nargin < 4 || isempty(windows), windows = [R.t(1) R.t(end)]; end
if nargin < 5, caption = repmat({''}, 1, size(windows, 1)); end
H = sp_anim('handles');
mp.movie = 0;  mp.gif = 0;
blk.DialogPrm(1).Data = mp;  blk.DialogPrm(2).Data = Inf;      % Inf = no real-time pacing
blk.InputPort(1).Data = R.pos(1, :).';  blk.InputPort(2).Data = R.legs(1, :).';
blk.InputPort(3).Data = R.phi(1, :).';  blk.CurrentTime = R.t(1);
close(findobj(0, 'type', 'figure', 'name', H.fig_name()));
H.Start(blk);
hFig = findobj(0, 'type', 'figure', 'name', H.fig_name());  hFig = hFig(1);
set(hFig, 'Position', [50 50 1000 620], 'Color', 'w');
cap = annotation(hFig, 'textbox', [0.55 0.925 0.42 0.05], 'String', '', 'EdgeColor', 'none', ...
                 'HorizontalAlignment', 'right', 'FontSize', 11, 'FontWeight', 'bold', 'Color', [0.2 0.2 0.2]);
vw = VideoWriter([out '.mp4'], 'MPEG-4');  vw.FrameRate = round(1/mp.dt_anim);  open(vw);
first = true;  nfr = 0;
for w = 1:size(windows, 1)
    for t = windows(w, 1):mp.dt_anim:windows(w, 2)
        k = find(R.t >= t - 1e-9, 1);
        if isempty(k), break; end
        blk.InputPort(1).Data = R.pos(k, :).';  blk.InputPort(2).Data = R.legs(k, :).';
        blk.InputPort(3).Data = R.phi(k, :).';  blk.CurrentTime = R.t(k);
        H.Update(blk);
        set(cap, 'String', caption{w});
        fr = getframe(hFig);
        writeVideo(vw, fr);
        [Im, map] = rgb2ind(fr.cdata, 128);
        if first
            imwrite(Im, map, [out '.gif'], 'gif', 'LoopCount', Inf, 'DelayTime', mp.dt_anim);  first = false;
        else
            imwrite(Im, map, [out '.gif'], 'gif', 'WriteMode', 'append', 'DelayTime', mp.dt_anim);
        end
        nfr = nfr + 1;
    end
end
close(vw);  H.Terminate(blk);  close(hFig);
fprintf('sp_anim_replay: %d frames -> %s.mp4 / .gif\n', nfr, out);
end
