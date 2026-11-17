function [label, cls] = sp_gait_label(G)
% SP_GAIT_LABEL  Gait name (and coarse class without numbers) from the lag metrics of sp_gait_class.
%   locked   : |drift| < 0.3 deg/s and circular std < 5 deg over the window
%   drifting : the lag still moves; the direction tells which pattern it is heading to
%              (in 2-D the physical offset obeys  d(offset)/dt ~ -A sin(2 lag):  drift*sin(2 lag) < 0
%              means the pair is closing in on in-step = pace / trot, > 0 on half a step apart)
% Reference lags (Lanini 2017): pace ~0, trot ~180, diagonal sequence ~225 deg (135 = mirror);
% 90 / 270 deg = half a step apart.
if G.fell
    label = sprintf('fell at %.1f s', G.t_end);  cls = 'fell';
elseif isnan(G.drift)
    label = 'too few strides';  cls = label;
elseif abs(G.drift) < 0.3 && G.lag_std < 5
    refs  = [0 180 225 135 90 270 360];
    names = {'pace', 'trot', 'diagonal sequence', 'diagonal sequence (mirror)', 'half-step', 'half-step', 'pace'};
    [d, j] = min(abs(G.lag_end - refs));
    if d <= 15, label = names{j}; else, label = sprintf('other (%.0f deg)', G.lag_end); end
    cls = ['locked: ' label];
else
    s = G.drift*sind(2*G.lag_end);
    if s < 0, to = 'in-step (pace/trot)'; else, to = 'half-step'; end
    label = sprintf('drifting to %s (%+.2f deg/s)', to, G.drift);
    cls = ['drifting to ' to];
end
end
