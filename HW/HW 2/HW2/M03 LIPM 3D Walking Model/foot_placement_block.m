function [xFP, yFP] = fcn(CM, vCM, goal, psi, mp)
% Turning-aware foot placement for 3-D LIPM walking.

%% LIPM terms
w = sqrt(mp.g / mp.z0);
C = cosh(w * mp.T_st);
S = sinh(w * mp.T_st);

D = mp.a * (C - 1)^2 ...
  + mp.b * (w * S)^2;

%% Desired state in local walking frame
x_des  = goal(1);
dx_des = goal(2);
y_des  = goal(3);
dy_des = goal(4);

%% Rotate current horizontal velocity into local frame
R = [cos(psi), -sin(psi); ...
     sin(psi),  cos(psi)];

v_local = R.' * vCM(1:2);

%% Nominal initial state
x0  = -x_des;
dx0 = v_local(1);

y0  = y_des;
dy0 = v_local(2);

%% Optimal placement correction
px = ...
    -mp.a * (C - 1) / D * ...
        (x_des - C*x0 - S/w*dx0) ...
    -mp.b * w*S / D * ...
        (dx_des - S*w*x0 - C*dx0);

py = ...
    -mp.a * (C - 1) / D * ...
        (y_des - C*y0 - S/w*dy0) ...
    -mp.b * w*S / D * ...
        (dy_des - S*w*y0 - C*dy0);

%% Nominal foot displacement in local walking frame
p_nominal_local = [x_des; -y_des];
p_local = p_nominal_local + [px; py];

%% Transform back to world coordinates
p_world = CM(1:2) + R * p_local;

xFP = p_world(1);
yFP = p_world(2);

end
