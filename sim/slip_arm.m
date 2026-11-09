function [Fw, Fo, Pdiss, L, Ld] = slip_arm(y, P, opt)
% SLIP_ARM  Horizontal spring-damper arms between the walkers and the object
% (Lanini 2017, Sec. 3.2, Eqs. 2-6).  Walker 1 is behind the object, walker 2 in front.
%
%   L1 = xo - x1          (walker 1 to rear handle)       F1 = ka (L1 - L01) + ba dL1/dt
%   L2 = x2 - xo          (front handle to walker 2)      F2 = ka (L2 - L02) + ba dL2/dt
%   walker 1:  +F1     object:  -F1 + F2     walker 2:  -F2      (x direction only)
%   dissipated power:  Pdiss = ba1 dL1^2 + ba2 dL2^2
%
% The object length Lt only shifts L0 and drops out (Lanini Sec. 3.2), so xo is
% simply the object position and opt.L0 holds the two rest lengths.

nW = numel(opt.L0);  Fw = zeros(1, max(nW,1));  Fo = 0;  Pdiss = 0;  L = [0 0];  Ld = [0 0];
if ~opt.coupled, return; end

ka = P.ka .* [1 1];  ba = P.ba .* [1 1];      % scalar or per-arm values
x1 = y(1);  xd1 = y(3);  x2 = y(5);  xd2 = y(7);
xo = y(9);  xod = y(10);

L  = [xo - x1,   x2 - xo];
Ld = [xod - xd1, xd2 - xod];
F  = ka .* (L - opt.L0) + ba .* Ld;

Fw    = [F(1), -F(2)];
Fo    = -F(1) + F(2);
Pdiss = sum(ba .* Ld.^2);
end
