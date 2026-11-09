function [val, term, dir] = slip_events(~, y, W, P, ~)
% SLIP_EVENTS  Four event functions per walker; their meaning depends on the phase.
%
%  single support (stance leg s):
%    1  TD    z - l0 sin(a0)      falling  -> swing leg lands at x + l0 cos(a0)   (Geyer 2006)
%    2  APEX  dz/dt               falling  -> highest CoM point (return-map section)
%    3  FAIL  F_s / k             falling  -> stance leg unloads: flight
%    4  FAIL  dx/dt               falling  -> walker turns backward
%  double support (trailing leg r, leading leg d):
%    1  TO    F_r / k             falling  -> trailing leg unloads and takes off
%    2  FAIL  F_d / k             falling  -> leading leg unloads (falls backward)
%    3  FAIL  dx/dt               falling  -> walker turns backward
%    4  FAIL  z - z_fail          falling  -> collapse
%  The ground can only push, so a leg leaves the ground when its force F reaches 0.
%  Stage 1 (no push-off, no leg damper): F = k (l0 - l), i.e. take-off at l = l0 (Geyer 2006).

nW  = numel(W);  I = slip_idx(nW);
val = zeros(4*nW, 1);  dir = zeros(4*nW, 1);  term = ones(4*nW, 1);

for i = 1:nW
    q = y(4*i-3 : 4*i);  phi = y(I.phi(i));
    c = W(i).contact;  f = W(i).foot;
    k = 4*(i-1);
    if sum(c) == 1                                   % single support
        s = find(c);
        Ls = slip_leg(q, phi, s, f(s), P);
        if W(i).apexArmed, ap = q(4); else, ap = 1; end
        val(k+1:k+4) = [q(2) - P.l0*sin(P.a0); ap; Ls.F / P.k; q(3)];
        dir(k+1:k+4) = [-1; -1; -1; -1];
    else                                             % double support
        [~, r] = min(f);  d = 3 - r;
        Lr = slip_leg(q, phi, r, f(r), P);
        Ld = slip_leg(q, phi, d, f(d), P);
        val(k+1:k+4) = [Lr.F / P.k; Ld.F / P.k; q(3); q(2) - P.z_fail];
        dir(k+1:k+4) = [-1; -1; -1; -1];
    end
end
end
