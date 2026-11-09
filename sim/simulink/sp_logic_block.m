function [legs, phi, stop, td] = sp_logic_block(pos, vel, mp)
% Gait logic + CPG  (code of the MATLAB Function block "Gait Logic + CPG" in cpg_pair.slx)
%
% Runs once per time step mp.dt (its inputs come through Zero-Order Holds).
% Keeps the discrete walker state in persistent variables and decides touchdown,
% take-off and falls; integrates the CPG phase of each walker (Euler).
%
% Inputs   pos = [x1 z1 x2 z2 xo],  vel = [dx1 dz1 dx2 dz2 dxo]
% Outputs  legs = [c11 c12 c21 c22, f11 f12 f21 f22, lam11 lam12 lam21 lam22]
%                 c = contact (1 stance), f = foot x, lam = leg rest length
%                 (walker i, leg j -> index 2*(i-1)+j)
%          phi  = CPG phase of walker 1 and 2, wrapped to [0, 2 pi)
%          stop = 0 walking, 1 flight in single support, 2 walked backward,
%                 3 leading leg lifted off, 4 collapsed
%          td   = leg that touched down in this step (0 = none), per walker
%
% Events (Geyer 2006; leg leaves the ground when its force reaches 0):
%   single support:  TD when z falls through l0 sin(a0); swing foot lands at x + l0 cos(a0)
%   double support:  TO of the trailing leg when its force F <= 0
% CPG (Owaki & Ishiguro 2017 Eq. 1; leg j phase theta_j = phi + (j-1) pi):
%   dphi/dt = omega - sigma sum_j N_j cos(theta_j),  N_j = vertical GRF [body weights]
%   touchdown of leg j:  theta_j <- th_TD  (Aoi & Tsuchiya 2007)

persistent c f ph zprev stopped

if isempty(c)
    c = [1 0; 1 0];                          % both walkers start on leg 1
    f = [mp.foot0(1) 0; mp.foot0(2) 0];
    ph = [mp.phi0(1); mp.phi0(2)];
    zprev = [pos(2); pos(4)];
    stopped = 0;
end

stop = stopped;
td = [0; 0];
zTD = mp.l0*sin(mp.a0);

if stopped == 0
    for i = 1:mp.nW
        x = pos(2*i-1);  z = pos(2*i);  xd = vel(2*i-1);  zd = vel(2*i);

        % leg forces with the current rest lengths
        F = [0 0];  N = [0 0];
        for j = 1:2
            if c(i,j) > 0.5
                [F(j), ez] = leg_force(x, z, xd, zd, f(i,j), ph(i) + (j-1)*pi, mp);
                N(j) = max(F(j), 0)*ez/(mp.m*mp.g);
            end
        end

        % CPG phase (Euler step)
        if mp.cpg > 0.5
            fb = N(1)*cos(ph(i)) + N(2)*cos(ph(i) + pi);
            ph(i) = mod(ph(i) + mp.dt*(mp.omega(i) - mp.sigma*fb), 2*pi);
        end

        nc = c(i,1) + c(i,2);
        if nc < 1.5                                   % ---- single support
            s = 1;  if c(i,2) > 0.5, s = 2; end
            w = 3 - s;
            if F(s) < 0
                stopped = 1;
            elseif xd <= 0
                stopped = 2;
            elseif z <= zTD && zprev(i) > zTD         % touchdown of the swing leg
                c(i,w) = 1;
                f(i,w) = x + mp.l0*cos(mp.a0);
                td(i) = w;
                if mp.cpg > 0.5
                    ph(i) = mod(mp.th_TD - (w-1)*pi, 2*pi);
                end
            end
        else                                          % ---- double support
            r = 1;  if f(i,2) < f(i,1), r = 2; end    % trailing leg
            d = 3 - r;
            if F(r) <= 0                              % take-off of the trailing leg
                c(i,r) = 0;
                f(i,r) = 0;
            elseif F(d) < 0
                stopped = 3;
            elseif xd <= 0
                stopped = 2;
            elseif z < mp.z_fail
                stopped = 4;
            end
        end
        zprev(i) = z;
    end
end

% outputs
legs = zeros(12, 1);
for i = 1:2
    for j = 1:2
        n = 2*(i-1) + j;
        legs(n) = c(i,j);
        legs(4+n) = f(i,j);
        legs(8+n) = mp.l0 + mp.dl*pushoff(ph(i) + (j-1)*pi, mp);
    end
end
phi = ph;
stop = stopped;
end

% ------------------------------------------------------------------------------
function [F, ez] = leg_force(x, z, xd, zd, foot, theta, mp)
% axial leg force  F = k (lambda - l) - c dl/dt   (positive = pushing)
dx = x - foot;
l  = sqrt(dx^2 + z^2);
ex = dx/l;  ez = z/l;
ld = ex*xd + ez*zd;
lam = mp.l0 + mp.dl*pushoff(theta, mp);
F = mp.k*(lam - l) - mp.c*ld;
end

function u = pushoff(theta, mp)
% push-off command in [0, 1]: sin^2 ramp from th_on, held until th_on + th_hold
u = 0;
if mp.cpg > 0.5 && mp.dl > 0
    s = mod(theta - mp.th_on, 2*pi);
    if s < mp.th_ramp
        u = sin(pi/2*s/mp.th_ramp)^2;
    elseif s < mp.th_hold
        u = 1;
    end
end
end
