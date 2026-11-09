function [s1, ok, info] = slip_apex_map(P, s0, E)
% SLIP_APEX_MAP  Apex return map of one walker (Geyer 2006, App. A(b)).
%
%   Passive walker (energy conserved), 2-D section:
%     s = [dx; y]            dx = CoM minus stance foot, y = CoM height, speed from E
%   CPG walker (Stage 2, energy not conserved), 4-D section:
%     s = [dx; y; xd; th]    th = CPG phase of the stance leg, mod 2 pi  (E unused)
%
%   s1 = state at the next single-support apex (on the other leg)
%   ok = false if the walker fails before reaching it

n = numel(s0);  s1 = NaN(n, 1);  ok = false;  info = struct('T', NaN, 'v', NaN, 'status', '');
try
    if n == 2
        W = slip_walker_at_apex(P, s0(1), s0(2), E);
    else
        W = slip_walker_at_apex(P, s0(1), s0(2), 1e4);   % placeholder speed, overwritten below
        W.xd = s0(3);  W.phi = s0(4);
    end
catch
    info.status = 'invalid apex';  return;
end
o = slip_sim(P, W, struct('tEnd', 5, 'nApexStop', 1));
info.status = o.status;
if ~strcmp(o.status, 'ok') || isempty(o.ev), return; end
a = o.ev(strcmp({o.ev.type}, 'APEX'));
if isempty(a), info.status = 'no apex'; return; end
a = a(end);
x = a.y(1);  z = a.y(2);
if n == 2
    s1 = [x - a.foot(a.leg); z];
else
    I = slip_idx(1);
    s1 = [x - a.foot(a.leg); z; a.y(3); mod(a.y(I.phi) + (a.leg - 1)*pi, 2*pi)];
end
ok = true;
info.T = a.t;                         % step time  (apex to apex)
info.v = (x - W.x) / a.t;             % average forward speed over the step
end
