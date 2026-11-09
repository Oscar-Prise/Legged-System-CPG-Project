function [sf, lam, info] = slip_find_fixed_point(P, s_guess, E)
% SLIP_FIND_FIXED_POINT  Period-1 fixed point s* = R(s*) of the apex map (slip_apex_map),
% by Newton's method with a finite-difference Jacobian, and the eigenvalues of dR/ds at s*.
%   Stable (a limit cycle) if all |lam| < 1  (Geyer 2006, App. A(c)).
%   s_guess = [dx; y] (passive, energy E) or [dx; y; xd; th] (CPG walker, E unused).

if nargin < 3, E = NaN; end
s = s_guess(:);  n = numel(s);  h = 1e-6;
sf = NaN(n, 1);  lam = NaN(n, 1);  info = struct('ok', false);
for it = 1:30
    [R, ok] = slip_apex_map(P, s, E);
    if ~ok, return; end
    G = wrap(R - s);
    J = zeros(n);
    for c = 1:n
        e = zeros(n, 1);  e(c) = h;
        [Rc, okc] = slip_apex_map(P, s + e, E);
        if ~okc, return; end
        J(:, c) = wrap(Rc - R) / h;
    end
    ds = -(J - eye(n)) \ G;
    s = s + ds;
    if norm(ds) < 1e-10, break; end
end
[R, ok, mi] = slip_apex_map(P, s, E);
if ~ok || norm(wrap(R - s)) > 1e-7, return; end
if n == 4, s(4) = mod(s(4), 2*pi); end
sf = s;  lam = eig(J);
info.ok = true;  info.T = mi.T;  info.v = mi.v;  info.J = J;  info.iter = it;
end

function d = wrap(d)
% phase differences (4th entry of the CPG section) wrapped to (-pi, pi]
if numel(d) == 4, d(4) = mod(d(4) + pi, 2*pi) - pi; end
end
