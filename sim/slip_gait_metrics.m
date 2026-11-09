function g = slip_gait_metrics(out, r)
% SLIP_GAIT_METRICS  Step timing, phase lag, apex return map and arm loss per step.
%
%   g.tTD{i}, g.legTD{i}   touchdown times and leg labels of walker i
%   g.Tstep{i}             step periods (TD to TD)
%   g.apex{i}              [t dx y] at every single-support apex
%   g.dApex{i}             |(dx,y)_{k+1} - (dx,y)_k|  (Lanini 2017 Sec. 4.1, periodicity check)
%   g.phi                  stride phase lag [deg] of walker 2 vs walker 1, homologous legs
%                          (0 = pace, 180 = trot, ~225 = diagonal sequence; Lanini 2017 Sec. 2.1, 2.3)
%   g.tPhi                 times (walker-1 leg-1 touchdowns) at which g.phi is evaluated
%   g.Ediss_step           arm energy dissipated during each walker-1 step [J]
%   g.tStep                start times of these steps

ev = out.ev;  nW = numel(out.W);
for i = 1:nW
    e = ev([ev.walker] == i & strcmp({ev.type}, 'TD'));
    g.tTD{i} = [e.t];  g.legTD{i} = [e.leg];
    g.Tstep{i} = diff(g.tTD{i});
    a = ev([ev.walker] == i & strcmp({ev.type}, 'APEX'));
    A = zeros(numel(a), 3);
    for n = 1:numel(a)
        A(n, :) = [a(n).t, a(n).y(4*i-3) - a(n).foot(a(n).leg), a(n).y(4*i-2)];
    end
    g.apex{i} = A;
    if size(A, 1) > 1, g.dApex{i} = sqrt(sum(diff(A(:, 2:3)).^2, 2)); else, g.dApex{i} = []; end
end

% stride phase lag between homologous legs (leg 1 of both walkers)
g.phi = [];  g.tPhi = [];
if nW == 2
    t1 = g.tTD{1}(g.legTD{1} == 1);  t2 = g.tTD{2}(g.legTD{2} == 1);
    for n = 1:numel(t1) - 1
        Ts = t1(n+1) - t1(n);                         % stride of walker 1
        [~, m] = min(abs(t2 - t1(n)));
        if isempty(m), continue; end
        g.phi(end+1)  = mod(360 * (t2(m) - t1(n)) / Ts, 360);
        g.tPhi(end+1) = t1(n);
    end
end

% arm dissipation per step of walker 1
tt = g.tTD{1};
g.tStep = tt(1:end-1);
g.Ediss_step = diff(interp1(r.t, r.Ediss, tt));
end
