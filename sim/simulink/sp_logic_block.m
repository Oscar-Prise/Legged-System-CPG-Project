function [legs, phi, stop, td, work, state] = sp_logic_block(pos, vel, mp)
% Gait logic + CPG  (code of the MATLAB Function block "Gait Logic and CPG" in cpg_pair.slx)
%
% Runs once per time step mp.dt (its inputs come through Zero-Order Holds).
% Keeps the discrete walker state in persistent variables and decides touchdown,
% take-off and falls; integrates the CPG phase of each walker (Euler) and, in the
% foot-mass model, the swing feet (semi-implicit Euler).
%
% Leg model switch  mp.legmodel:
%   1 = leg damper:  F = k (lambda - l) - c dl/dt in stance; massless swing leg held at a0;
%                    touchdown when z falls through l0 sin(a0)                 (Geyer 2006)
%   2 = foot mass:   no leg damper (set c = 0); a point foot of mass mf on each massless
%                    spring leg.  In swing the CPG phase drives the foot along a target path
%                    (hip servo); touchdown when the foot reaches the ground; the foot stops
%                    there, losing  dE = mf |v_foot|^2 / 2  (the energy sink)
%
% Inputs   pos = [x1 z1 x2 z2 xo],  vel = [dx1 dz1 dx2 dz2 dxo]
% Outputs  legs (36) for walker i, leg j -> n = 2*(i-1)+j:
%            1-4   c    contact (1 stance)          5-8   f    stance foot x
%            9-12  lam  leg rest length             13-16 u    push-off command [0..1]
%            17-20 xf   foot x (swing or stance)    21-24 zf   foot height
%            25-28 Fh   [Fx1 Fz1 Fx2 Fz2] force of the swing feet on the bodies
%            29-32 tTD  time of the last touchdown  33-36 tTO  time of the last take-off
%          phi  = CPG phase of walker 1 and 2, wrapped to [0, 2 pi)
%          stop = 0 walking, 1 flight in single support, 2 walked backward,
%                 3 leading leg lifted off, 4 collapsed
%          td   = leg that touched down in this step (0 = none), per walker
%          work = [push-off work, hip (swing) work, leg losses at events, swing-foot energy] [J]
%                 leg losses at events = foot impacts (foot model) + spring energy left in
%                 the leg when it lifts off (damper model: the damper cancels the spring force)
%          state = discrete state of walker 1 (1-25) and 2 (26-50), each
%                 [c(2) f(2) lb(2) lamp(2) xf(2) zf(2) vxf(2) vzf(2) xTO(2) thTO(2) tTD(2) tTO(2) phi]
%                 (used to start a run from the state of an earlier run, see sp_start_state.m)
%
% Body parameters are per walker: mp.m, l0, k, a0, c, dl, mf, Ksw, Dsw, h_sw, omega are 1x2.
% Start: walker i starts at an apex on leg 1 (mp.foot0, mp.phi0) unless mp.init_use(i) = 1,
% then from mp.init (same layout as 'state').
%
% CPG (Owaki & Ishiguro 2017 Eq. 1; leg j phase theta_j = phi + (j-1) pi):
%   dphi/dt = omega - sigma sum_j N_j cos(theta_j),  N_j = vertical GRF [body weights]
%   touchdown of leg j:  theta_j <- th_TD  (Aoi & Tsuchiya 2007)
% Swing (foot model): progress s = (theta - theta_TO)/(th_TD - theta_TO) in [0, 1];
%   foot target  r*(s) = (1 - b) r_TO + b (r_hip + l0 [cos a0, -sin a0]) + h_sw sin^2(pi s) e_z,
%   b = 3s^2 - 2s^3, r_TO = where the foot left the ground.  The target starts at rest on the
%   ground and ends at the angle of attack a0, moving with the hip (no kick at take-off).
%   hip servo  F = Ksw (r* - r) + Dsw (v* - v) + mf g, reaction -F on the body

persistent c f ph zprev stopped lb lamp xf zf vxf vzf xTO thTO tTD tTO nstep W

if isempty(c)
    c = [1 0; 1 0];                          % both walkers start on leg 1
    f = [mp.foot0(1) 0; mp.foot0(2) 0];
    ph = [mp.phi0(1); mp.phi0(2)];
    zprev = [pos(2); pos(4)];
    stopped = 0;
    lb = [mp.l0(1) mp.l0(1); mp.l0(2) mp.l0(2)];   % stance rest length before push-off
    lamp = lb;                                     % rest length in effect (held output)
    xf = zeros(2, 2);  zf = zeros(2, 2);  vxf = zeros(2, 2);  vzf = zeros(2, 2);
    xTO = zeros(2, 2);  thTO = zeros(2, 2);
    tTD = [0 -1; 0 -1];  tTO = -ones(2, 2);
    nstep = 0;
    W = zeros(4, 1);
    for i = 1:2
        xf(i,1) = f(i,1);
        % swing leg 2 starts half way through its swing (as at a mid-stance apex)
        th2 = mod(ph(i) + pi, 2*pi);
        thTO(i,2) = mod(2*th2 - mp.th_TD, 2*pi);
        xTO(i,2) = pos(2*i-1) - mp.l0(i)*cos(mp.a0(i));      % as if it left the ground there
        sd = mp.omega(i)/max(mod(mp.th_TD - thTO(i,2), 2*pi), 0.5);
        [xf(i,2), zf(i,2), vxf(i,2), vzf(i,2)] = swing_target(0.5, sd, xTO(i,2), ...
            pos(2*i-1), pos(2*i), vel(2*i-1), vel(2*i), mp.l0(i), mp.a0(i), mp.h_sw(i));
        if mp.init_use(i) > 0.5                        % start from a given state instead
            S = mp.init(25*(i-1) + (1:25));
            c(i,:) = S(1:2);     f(i,:) = S(3:4);     lb(i,:) = S(5:6);    lamp(i,:) = S(7:8);
            xf(i,:) = S(9:10);   zf(i,:) = S(11:12);  vxf(i,:) = S(13:14); vzf(i,:) = S(15:16);
            xTO(i,:) = S(17:18); thTO(i,:) = S(19:20); tTD(i,:) = S(21:22); tTO(i,:) = S(23:24);
            ph(i) = S(25);
        end
    end
end

t = nstep*mp.dt;
td = [0; 0];
Fh = zeros(4, 1);
foot = mp.legmodel > 1.5;
osc = mp.cpg > 0.5 || foot;                  % the foot model always needs the phase (swing timing)

if stopped == 0
    for i = 1:mp.nW
        x = pos(2*i-1);  z = pos(2*i);  xd = vel(2*i-1);  zd = vel(2*i);
        m = mp.m(i);  l0 = mp.l0(i);  a0 = mp.a0(i);  mf = mp.mf(i);
        zTD = l0*sin(a0);

        % leg forces with the rest lengths in effect during the last step
        F = [0 0];  N = [0 0];
        for j = 1:2
            if c(i,j) > 0.5
                [F(j), ez] = leg_force(x, z, xd, zd, f(i,j), lamp(i,j), mp.k(i), mp.c(i));
                N(j) = max(F(j), 0)*ez/(m*mp.g);
            end
        end

        % CPG phase (Euler step)
        dph = 0;
        if osc
            dph = mp.omega(i) - mp.sigma*(N(1)*cos(ph(i)) + N(2)*cos(ph(i) + pi));
            ph(i) = mod(ph(i) + mp.dt*dph, 2*pi);
        end

        nc = c(i,1) + c(i,2);
        if nc < 1.5                                   % ---- single support
            s = 1;  if c(i,2) > 0.5, s = 2; end
            w = 3 - s;
            if F(s) < 0
                stopped = 1;
            elseif xd <= 0
                stopped = 2;
            elseif z < mp.z_fail*l0
                stopped = 4;
            elseif ~foot && z <= zTD && zprev(i) > zTD                % touchdown, leg damper model
                c(i,w) = 1;
                f(i,w) = x + l0*cos(a0);
                lb(i,w) = l0;  lamp(i,w) = l0;
                td(i) = w;  tTD(i,w) = t;
                if osc, ph(i) = mod(mp.th_TD - (w-1)*pi, 2*pi); end
            elseif foot && swing_progress(ph(i) + (w-1)*pi, thTO(i,w), mp) > 0.5 ...
                    && zf(i,w) <= 0 && vzf(i,w) < 0                   % touchdown, foot model
                W(3) = W(3) + 0.5*mf*(vxf(i,w)^2 + vzf(i,w)^2);      % foot stops: impact loss
                c(i,w) = 1;
                f(i,w) = xf(i,w);  zf(i,w) = 0;  vxf(i,w) = 0;  vzf(i,w) = 0;
                lb(i,w) = sqrt((x - xf(i,w))^2 + z^2);  lamp(i,w) = lb(i,w);   % no force jump
                td(i) = w;  tTD(i,w) = t;
                ph(i) = mod(mp.th_TD - (w-1)*pi, 2*pi);
            end
        else                                          % ---- double support
            r = 1;  if f(i,2) < f(i,1), r = 2; end    % trailing leg
            d = 3 - r;
            if F(r) <= 0                              % take-off of the trailing leg
                c(i,r) = 0;  tTO(i,r) = t;
                lr = sqrt((f(i,r) - x)^2 + z^2);      % spring energy left when the damper
                if lamp(i,r) > lr                     % force cancels the spring: lost
                    W(3) = W(3) + 0.5*mp.k(i)*(lamp(i,r) - lr)^2;
                end
                % the swing starts from the foot at rest on the ground
                thTO(i,r) = mod(ph(i) + (r-1)*pi, 2*pi);
                xTO(i,r) = f(i,r);
                xf(i,r) = f(i,r);  zf(i,r) = 0;  vxf(i,r) = 0;  vzf(i,r) = 0;
                f(i,r) = 0;
            elseif F(d) < 0
                stopped = 3;
            elseif xd <= 0
                stopped = 2;
            elseif z < mp.z_fail*l0
                stopped = 4;
            end
        end
        zprev(i) = z;

        % swing legs
        for j = 1:2
            if c(i,j) < 0.5
                if foot                               % foot mass driven by the hip servo
                    th = ph(i) + (j-1)*pi;
                    sw = swing_progress(th, thTO(i,j), mp);
                    sd = 0;
                    if sw < 1, sd = dph/max(mod(mp.th_TD - thTO(i,j), 2*pi), 0.5); end
                    [xs, zs, vxs, vzs] = swing_target(sw, sd, xTO(i,j), x, z, xd, zd, l0, a0, mp.h_sw(i));
                    Fx = mp.Ksw(i)*(xs - xf(i,j)) + mp.Dsw(i)*(vxs - vxf(i,j));
                    Fz = mp.Ksw(i)*(zs - zf(i,j)) + mp.Dsw(i)*(vzs - vzf(i,j)) + mf*mp.g;
                    vx0 = vxf(i,j);  vz0 = vzf(i,j);
                    vxf(i,j) = vxf(i,j) + mp.dt*Fx/mf;
                    vzf(i,j) = vzf(i,j) + mp.dt*(Fz/mf - mp.g);
                    W(2) = W(2) + mp.dt*(Fx*((vx0 + vxf(i,j))/2 - xd) ...      % hip work
                                       + Fz*((vz0 + vzf(i,j))/2 - zd));
                    xf(i,j) = xf(i,j) + mp.dt*vxf(i,j);
                    zf(i,j) = zf(i,j) + mp.dt*vzf(i,j);
                    if sw < 0.5 && zf(i,j) < 0        % early swing: no scuffing through the floor
                        zf(i,j) = 0;  if vzf(i,j) < 0, vzf(i,j) = 0; end
                    end
                    Fh(2*i-1) = Fh(2*i-1) - Fx;       % reaction on the body
                    Fh(2*i)   = Fh(2*i)   - Fz;
                else                                  % massless leg held at the angle of attack
                    xf(i,j) = x + l0*cos(a0);  zf(i,j) = z - l0*sin(a0);
                end
            else
                xf(i,j) = f(i,j);  zf(i,j) = 0;
            end
        end
    end
end

% push-off: new rest lengths and the work the leg actuator does by changing them
u = zeros(2, 2);
for i = 1:mp.nW
    for j = 1:2
        if c(i,j) > 0.5
            u(i,j) = pushoff(ph(i) + (j-1)*pi, mp.dl(i), mp);
            lam = lb(i,j) + mp.dl(i)*u(i,j);
            l = sqrt((pos(2*i-1) - f(i,j))^2 + pos(2*i)^2);
            dlam = lam - lamp(i,j);
            W(1) = W(1) + mp.k(i)*(lamp(i,j) - l)*dlam + 0.5*mp.k(i)*dlam^2;   % spring energy change
            lamp(i,j) = lam;
        end
    end
end

% energy of the swing feet (for the energy check)
W(4) = 0;
if foot
    for i = 1:mp.nW
        for j = 1:2
            if c(i,j) < 0.5
                W(4) = W(4) + 0.5*mp.mf(i)*(vxf(i,j)^2 + vzf(i,j)^2) + mp.mf(i)*mp.g*zf(i,j);
            end
        end
    end
end

% outputs
legs = zeros(36, 1);
for i = 1:2
    for j = 1:2
        n = 2*(i-1) + j;
        legs(n)    = c(i,j);
        legs(4+n)  = f(i,j);
        legs(8+n)  = lamp(i,j);
        legs(12+n) = u(i,j);
        legs(16+n) = xf(i,j);
        legs(20+n) = zf(i,j);
        legs(28+n) = tTD(i,j);
        legs(32+n) = tTO(i,j);
    end
end
legs(25:28) = Fh;
phi = ph;
stop = stopped;
work = W;
state = zeros(50, 1);
for i = 1:2
    state(25*(i-1) + (1:25)) = [c(i,:), f(i,:), lb(i,:), lamp(i,:), xf(i,:), zf(i,:), ...
        vxf(i,:), vzf(i,:), xTO(i,:), thTO(i,:), tTD(i,:), tTO(i,:), ph(i)].';
end
nstep = nstep + 1;
end

% ------------------------------------------------------------------------------
function [F, ez] = leg_force(x, z, xd, zd, foot, lam, k, c)
% axial leg force  F = k (lambda - l) - c dl/dt   (positive = pushing)
dx = x - foot;
l  = sqrt(dx^2 + z^2);
ex = dx/l;  ez = z/l;
ld = ex*xd + ez*zd;
F = k*(lam - l) - c*ld;
end

function u = pushoff(theta, dl, mp)
% push-off command in [0, 1]: sin^2 ramp from th_on, held until th_on + th_hold
u = 0;
if mp.cpg > 0.5 && dl > 0
    s = mod(theta - mp.th_on, 2*pi);
    if s < mp.th_ramp
        u = sin(pi/2*s/mp.th_ramp)^2;
    elseif s < mp.th_hold
        u = 1;
    end
end
end

function s = swing_progress(theta, thTO, mp)
% swing progress in [0, 1] from the leg phase: 0 at take-off, 1 at the phase th_TD
D = max(mod(mp.th_TD - thTO, 2*pi), 0.5);
s = min(mod(theta - thTO, 2*pi)/D, 1);
end

function [xs, zs, vxs, vzs] = swing_target(s, sd, xTO, x, z, xd, zd, l0, a0, h)
% foot target (world frame) and its velocity; sd = ds/dt
b  = 3*s^2 - 2*s^3;            bs = 6*s - 6*s^2;
xe = x + l0*cos(a0);           ze = z - l0*sin(a0);           % landing point at a0
xs = (1 - b)*xTO + b*xe;
zs = b*ze + h*sin(pi*s)^2;
vxs = bs*sd*(xe - xTO) + b*xd;
vzs = bs*sd*ze + b*zd + h*pi*sin(2*pi*s)*sd;
end
