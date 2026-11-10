function [acc, grf, Farm, Ploss] = sp_eom_block(pos, vel, legs, mp)
% Equations of motion  (code of the MATLAB Function block "Leg + Arm Forces" in cpg_pair.slx)
%
% Inputs   pos = [x1 z1 x2 z2 xo],  vel = [dx1 dz1 dx2 dz2 dxo],  legs from the gait logic
%          (uses contact 1-4, stance foot 5-8, rest length 9-12, swing-foot force 25-28)
% Outputs  acc   = [ddx1 ddz1 ddx2 ddz2 ddxo]
%          grf   = [Fx1 Fz1 Fx2 Fz2]   ground reaction force on each walker (sum of its legs)
%          Farm  = [F1 F2]             arm forces (positive = arm stretched)
%          Ploss = [arm damper power, leg damper power]
%
% Stance leg j of walker i (Geyer 2006 App. A; leg damper + push-off from Stage 2):
%   F_j = max(0, k (lambda_j - l_j) - c dl_j/dt)   along the leg, foot -> CoM
%   m ddx = sum_j F_j ex_j + F_arm,i + Fh_x  m ddz = sum_j F_j ez_j - m g + Fh_z
%   Fh = force of the swing feet on the body (foot-mass model only, else 0)
%   m, k, c are per walker (mp.m(i), mp.k(i), mp.c(i))
% Arms, horizontal only (Lanini 2017 Eqs. 2-6; walker 1 behind, walker 2 in front):
%   L1 = xo - x1,  L2 = x2 - xo,  F_a = ka (L_a - L0_a) + ba dL_a/dt
%   walker 1: +F1,  object: mt ddxo = -F1 + F2,  walker 2: -F2

acc = zeros(5, 1);  grf = zeros(4, 1);  Farm = zeros(2, 1);  Ploss = zeros(2, 1);

% arms
Fa = [0; 0];
if mp.coupled > 0.5 && mp.nW > 1.5
    L  = [pos(5) - pos(1); pos(3) - pos(5)];
    Ld = [vel(5) - vel(1); vel(3) - vel(5)];
    Fa = mp.ka*(L - mp.L0(:)) + mp.ba*Ld;
    Farm = Fa;
    Ploss(1) = mp.ba*(Ld(1)^2 + Ld(2)^2);
    acc(5) = (-Fa(1) + Fa(2))/mp.mt;
end
Fw = [Fa(1); -Fa(2)];

% walkers
for i = 1:mp.nW
    x = pos(2*i-1);  z = pos(2*i);  xd = vel(2*i-1);  zd = vel(2*i);
    Fx = 0;  Fz = 0;
    for j = 1:2
        n = 2*(i-1) + j;
        if legs(n) > 0.5
            dx = x - legs(4+n);
            l  = sqrt(dx^2 + z^2);
            ex = dx/l;  ez = z/l;
            ld = ex*xd + ez*zd;
            F  = mp.k(i)*(legs(8+n) - l) - mp.c(i)*ld;
            if F < 0, F = 0; end                 % the ground can only push
            Fx = Fx + F*ex;
            Fz = Fz + F*ez;
            if F > 0, Ploss(2) = Ploss(2) + mp.c(i)*ld^2; end
        end
    end
    grf(2*i-1) = Fx;  grf(2*i) = Fz;
    acc(2*i-1) = (Fx + Fw(i) + legs(24+2*i-1))/mp.m(i);
    acc(2*i)   = (Fz + legs(24+2*i))/mp.m(i) - mp.g;
end
end
