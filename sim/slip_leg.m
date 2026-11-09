function L = slip_leg(q, phi, j, foot, P)
% SLIP_LEG  Force of stance leg j of one walker.
%
%   q = [x z xd zd] of the CoM, phi = CPG phase of the walker, foot = foot position.
%   Leg phase theta_j = phi + (j-1) pi  (the two legs are half a stride apart).
%
%   l      = |CoM - foot|,        ld = dl/dt  (positive = leg extending)
%   lambda = l0 + dl * u(theta_j)                         (CPG push-off, Stage 2)
%   F      = k (lambda - l) - c ld                         (spring + leg damper, along the leg)
%   Stage 1 (dl = 0, c = 0):  F = k (l0 - l)  ->  Geyer 2006 / Lanini 2017.

dx = q(1) - foot;  z = q(2);
l  = sqrt(dx^2 + z^2);
L.ex = dx / l;  L.ez = z / l;                 % unit vector foot -> CoM
L.l  = l;
L.ld = L.ex*q(3) + L.ez*q(4);
L.th = phi + (j - 1)*pi;
[u, du] = slip_pushoff(L.th, P);
L.lam  = P.l0 + P.dl*u;
L.dlam = P.dl*du;                             % d lambda / d theta
L.Fs   = P.k*(L.lam - l);                     % spring force
L.F    = L.Fs - P.c*L.ld;                     % total leg force (>= 0 while in stance)
end
