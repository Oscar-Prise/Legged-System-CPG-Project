%% check_left_turn_radius.m
% Fit a circle to the simulated CoM trajectory from the left-quarter-turn
% test and report the effective turning radius.
%
% Expected workspace variables:
%   time : simulation time [s]
%   CoM  : center-of-mass position, columns [x y z] [m]
%   mp   : parameter struct containing mp.T_st and mp.lstep
%
% Run this AFTER:
%   clearvars
%   m07_params_A2
%   sim('m07_A2')
%
% with:
%   mode = 'left_quarter';

%% Basic checks
assert(exist('time','var') == 1, 'Variable "time" was not found in the workspace.');
assert(exist('CoM','var')  == 1, 'Variable "CoM" was not found in the workspace.');
assert(exist('mp','var')   == 1, 'Variable "mp" was not found in the workspace.');

% Handle either plain arrays or timeseries objects
if isa(time, 'timeseries')
    t = time.Data(:);
else
    t = time(:,1);
end

if isa(CoM, 'timeseries')
    com_data = CoM.Data;
else
    com_data = CoM;
end

x = com_data(:,1);
y = com_data(:,2);

%% Select the turning portion
% Exclude the first stance phase to reduce start-up transient effects.
use = (t >= mp.T_st);

xf = x(use);
yf = y(use);

assert(numel(xf) >= 3, 'Not enough CoM samples are available for circle fitting.');

%% Algebraic least-squares circle fit
% Circle:
%   (x-xc)^2 + (y-yc)^2 = R^2
%
% Rearranged:
%   2*xc*x + 2*yc*y + c = x^2 + y^2
%
% where:
%   c = R^2 - xc^2 - yc^2

A = [2*xf, 2*yf, ones(size(xf))];
b = xf.^2 + yf.^2;

q = A \ b;

xc = q(1);
yc = q(2);
Rfit = sqrt(q(3) + xc^2 + yc^2);

%% Nominal commanded radius for comparison
% For the left-quarter test, goal generation uses 10 steps over 90 deg.
Nturn = 10;
dpsi = (pi/2) / Nturn;
Rnom = mp.lstep / dpsi;

radius_error = Rfit - Rnom;
radius_error_pct = 100 * radius_error / Rnom;

%% Print results
fprintf('\n=== Left-Turn Radius Check ===\n');
fprintf('Fitted CoM turning radius = %.4f m\n', Rfit);
fprintf('Nominal commanded radius  = %.4f m\n', Rnom);
fprintf('Difference                 = %+.4f m (%+.2f%%)\n', ...
        radius_error, radius_error_pct);
fprintf('Circle center              = (%.4f, %.4f) m\n', xc, yc);

%% Plot fit for visual verification
theta = linspace(0, 2*pi, 500);
x_circle = xc + Rfit*cos(theta);
y_circle = yc + Rfit*sin(theta);

figure;
plot(x, y, 'k-', 'LineWidth', 1.5);
hold on;
plot(x_circle, y_circle, '--', 'LineWidth', 1.2);
plot(xc, yc, 'x', 'MarkerSize', 10, 'LineWidth', 1.5);

axis equal;
grid on;
xlabel('x [m]');
ylabel('y [m]');
title(sprintf('Left-Turn CoM Circle Fit: R = %.3f m', Rfit));
legend('Simulated CoM trajectory', 'Fitted circle', 'Fitted center', ...
       'Location', 'best');

%% Leave useful values in the base workspace
left_turn_Rfit = Rfit;
left_turn_Rnom = Rnom;
left_turn_circle_center = [xc, yc];
