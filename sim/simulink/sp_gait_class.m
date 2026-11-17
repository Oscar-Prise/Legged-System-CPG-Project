function G = sp_gait_class(R, mp, tEnd, win)
% SP_GAIT_CLASS  Stride phase lag, lock test and gait label from a sp_fixedstep_check run.
%
%   G = sp_gait_class(R, mp, tEnd)        window = last 10 s of the run
%   G = sp_gait_class(R, mp, tEnd, win)   window = last win seconds
%
% Stride lag of walker 2 vs walker 1 from their leg-1 touchdowns (as in plot_cpg_pair_results):
%   lag(n) = 360 * (t_TD2 nearest to t_TD1(n) - t_TD1(n)) / stride_1(n)   [deg, 0..360)
% Reference lags (Lanini 2017 Sec. 2.2.5): pace ~0, trot ~180, diagonal sequence ~225 deg
% (135 = its mirror).  90 / 270 = half a step apart (no name in Lanini).
% In 2-D the body motion only depends on the "physical offset"
%   p = min(mod(lag, 180), 180 - mod(lag, 180))  in [0, 90] deg   (0 = in step, 90 = half a step)
% because swapping the leg labels of one walker adds 180 deg and swapping the walkers negates the lag.
%
% G fields: fell, stop, t_end, tl, lag (series), lag_end (circular mean over the window),
%           lag_std, drift [deg/s] (slope of the unwrapped lag over the window), locked
%           (|drift| < 0.3 deg/s and lag_std < 5 deg; label from sp_gait_label),
%           offset (physical offset), label, nstride, T1 (stride time), v1 (speed of walker 1),
%           Earm, Wpush (per stride, over the window)

if nargin < 4, win = 10; end
G.fell = R.stop > 0;  G.stop = R.stop;  G.t_end = R.t_end;
a = R.td(R.td(:,2) == 1 & R.td(:,3) == 1, 1);        % leg-1 touchdowns, walker 1
b = R.td(R.td(:,2) == 2 & R.td(:,3) == 1, 1);        % leg-1 touchdowns, walker 2
lag = [];  tl = [];
for n = 1:numel(a) - 1
    if isempty(b), break; end
    [~, m] = min(abs(b - a(n)));
    lag(end+1) = mod(360*(b(m) - a(n))/(a(n+1) - a(n)), 360);   %#ok<AGROW>
    tl(end+1) = a(n);                                           %#ok<AGROW>
end
G.tl = tl(:);  G.lag = lag(:);  G.nstride = numel(a) - 1;
G.lag_end = NaN;  G.lag_std = NaN;  G.drift = NaN;  G.locked = false;  G.offset = NaN;
G.T1 = NaN;  G.v1 = NaN;  G.Earm = NaN;  G.Wpush = NaN;

t0 = max(R.t_end - win, 0);
k = find(tl >= t0);
if numel(k) >= 3
    lw = lag(k);  tw = tl(k);
    lu = rad2deg(unwrap(deg2rad(lw)));
    pf = polyfit(tw, lu, 1);
    G.drift = pf(1);
    G.lag_end = mod(rad2deg(angle(mean(exp(1i*deg2rad(lw))))), 360);
    G.lag_std = rad2deg(sqrt(-2*log(max(abs(mean(exp(1i*deg2rad(lw)))), 1e-12))));
    p = mod(G.lag_end, 180);  G.offset = min(p, 180 - p);
    ka = find(a >= t0);
    if numel(ka) >= 2
        G.T1 = mean(diff(a(ka)));
        x1 = interp1(R.t, R.pos(:,1), [a(ka(1)) a(ka(end))]);
        G.v1 = diff(x1)/(a(ka(end)) - a(ka(1)));
        e = interp1(R.t, R.Eloss(:,1), [a(ka(1)) a(ka(end))]);
        G.Earm = diff(e)/(numel(ka) - 1);
        w = interp1(R.t, R.work(:,1), [a(ka(1)) a(ka(end))]);
        G.Wpush = diff(w)/(numel(ka) - 1);
    end
end

G.locked = ~G.fell && abs(G.drift) < 0.3 && G.lag_std < 5;
G.label = sp_gait_label(G);
G.tEnd = tEnd;
end
