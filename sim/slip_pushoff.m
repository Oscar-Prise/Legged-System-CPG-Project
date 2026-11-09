function [u, du] = slip_pushoff(theta, P)
% SLIP_PUSHOFF  Push-off command u(theta) in [0, 1] and du/dtheta for a leg with phase theta.
%
%   The leg's rest length is  lambda = l0 + dl * u(theta).
%   u ramps up as sin^2 over [th_on, th_on + th_ramp], then holds 1 until th_on + th_hold
%   (the leg has left the ground by then) and is 0 otherwise.  At touchdown the leg
%   phase is reset to th_TD, which lies outside the window, so every leg lands at l0.

u = 0;  du = 0;
if ~P.cpg || P.dl == 0, return; end
s = mod(theta - P.th_on, 2*pi);
if s < P.th_ramp
    u  = sin(pi/2 * s / P.th_ramp)^2;
    du = (pi/2 / P.th_ramp) * sin(pi * s / P.th_ramp);
elseif s < P.th_hold
    u = 1;
end
end
