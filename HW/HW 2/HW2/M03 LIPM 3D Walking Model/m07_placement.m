%
% Foot Placement in 3-D Linear Inverted Pendulum
%
% hgeyer, Sep 2025
%
clc 

% model params
g = 9.81; % [m/s^2]

% walking parameters
v_des = 1.0; % [m/s]
z0 = 0.8;
step_len = 0.5;  % [m]
step_wdth = 0.3;  % [m]
stance_time = 0.6; % [s]


% weight terms of cost function
a = 2;  % position weight
b = 1;  % speed weight

% precompute LIPM math terms
w = sqrt(g / z0);
C = cosh(w * stance_time);
S = sinh(w * stance_time);
D = a * (C-1)^2 + b * (w * S)^2;

% desired final states
x_des = step_len / 2;
dx_des = v_des;    

Rleg_flag = 1;
if Rleg_flag
    y_des = step_wdth / 2; 
else
    y_des = -step_wdth / 2;
end
dy_des = y_des * w * (C - 1) / S; 

% initial state
x0 = - x_des;
dx0 = dx_des;
y0 = y_des;
dy0 = -dy_des;

% optimal foot placement given cost terms a and b
px = - a * (C - 1) / D * (x_des - C * x0 - S / w * dx0) ...
  - b * w * S / D * (dx_des - S * w * x0 - C * dx0);

py = - a * (C - 1) / D * (y_des - C * y0 - S / w * dy0) ...
  - b * w * S / D * (dy_des - S * w * y0 - C * dy0);

% integration of LIPM dynamics for double check
dt = 0.001;

tVec = 0:dt:stance_time;
xVec = nan(1, length(tVec));
dxVec = nan(1, length(tVec));
yVec = nan(1, length(tVec));
dyVec = nan(1, length(tVec));

x = x0;
dx = dx0;
y = y0;
dy = dy0;
for i = 1:length(tVec)

  xVec(i) = x;
  dxVec(i) = dx;

  ddx = g / z0 * (x - px);
  dx = dx + ddx * dt;
  x = x + dx * dt;

  yVec(i) = y;
  dyVec(i) = dy;

  ddy = g / z0 * (y - py);
  dy = dy + ddy * dt;
  y = y + dy * dt;


end


figure(1), clf

subplot(211)
hold on
plot(tVec, xVec)
plot(tVec, yVec)
grid on
xlabel('time (s)')
ylabel('position (m)')
legend('fore-aft (x)', 'mediolateral (y)')

subplot(212)
hold on
plot(tVec, dxVec)
plot(tVec, dyVec)
grid on
xlabel('time(s)')
ylabel('velocity (m/s)')
legend('fore-aft (x)', 'mediolateral (y)')

figure(2), clf
hold on
plot(xVec, yVec)
plot(0, 0, 'ko', 'MarkerSize', 10)
plot(px, py, 'bo', 'MarkerSize', 10)
grid on
axis equal padded
xlabel('fore-aft position (m)')
ylabel('mediolaeral position (m)')
legend('CoM trajectory', 'nominal foot placement', 'corrected foot placement')

