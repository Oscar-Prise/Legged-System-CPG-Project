%% Problem 1.1 - CE force-length and force-velocity relations
% Surface plot of M(lCE, vCE) = f_l(lCE) * f_v(vCE)
% for lCE in [0, 2*lopt] and vCE in [-vmax, +vmax].
%
% Sign convention: vCE < 0 -> shortening (concentric)
%                  vCE > 0 -> lengthening (eccentric)

clear; close all; clc;

%% Muscle parameters (from assignment)
mp.Fmax = 6000;          % [N]   max isometric force
mp.lopt = 0.08;          % [m]   optimum CE length (8 cm)
mp.w    = 0.56;          % [lopt] width of force-length curve (w = 0.56*lopt)
mp.c    = 0.05;          % []    remaining force at lCE = lopt*(1 +/- w)
mp.vmax = 12 * mp.lopt;  % [m/s] max contraction velocity (12 lopt/s)
mp.N    = 1.5;           % []    eccentric force enhancement (Necc)
mp.K    = 5;             % []    curvature (shape) factor of f_v

%% Grid
nL = 101; nV = 101;
lCE = linspace(0, 2*mp.lopt, nL);          % [m]
vCE = linspace(-mp.vmax, mp.vmax, nV);     % [m/s]
[L, V] = meshgrid(lCE, vCE);

%% Evaluate relations
FL = f_length(L, mp);
FV = f_velocity(V, mp);
M  = FL .* FV;

%% Plots: individual relations
figure('Name','CE relations','Position',[100 100 900 350]);
subplot(1,2,1);
plot(lCE/mp.lopt, f_length(lCE, mp), 'LineWidth', 2); grid on;
xlabel('l_{CE} / l_{opt}'); ylabel('f_l^a');
title('Active force-length relation');

subplot(1,2,2);
plot(vCE/mp.vmax, f_velocity(vCE, mp), 'LineWidth', 2); grid on; hold on;
xline(0, '--k'); yline(1, ':k');
xlabel('v_{CE} / v_{max}  (<0 shortening, >0 lengthening)'); ylabel('f_v');
title('Force-velocity relation');

%% Surface plot of M = f_l * f_v
figure('Name','M(lCE, vCE)');
surf(L/mp.lopt, V/mp.vmax, M, 'EdgeColor', 'none'); hold on;
contour3(L/mp.lopt, V/mp.vmax, M, 15, 'k');
colormap(parula); colorbar;
xlabel('l_{CE} / l_{opt}');
ylabel('v_{CE} / v_{max}');
zlabel('M = f_l^a \cdot f_v');
title('Normalized CE force capacity M(l_{CE}, v_{CE})');
view(-135, 30); grid on;

% Annotate the two hiking cases at l_CE = l_opt, |v| = 0.3 vmax
vh = 0.3 * mp.vmax;
Mup   = f_length(mp.lopt, mp) * f_velocity(-vh, mp);  % uphill: shortening
Mdown = f_length(mp.lopt, mp) * f_velocity( vh, mp);  % downhill: lengthening
plot3(1, -0.3, Mup,   'ro', 'MarkerFaceColor','r', 'MarkerSize', 8);
plot3(1,  0.3, Mdown, 'bo', 'MarkerFaceColor','b', 'MarkerSize', 8);
legend({'M','contours','uphill (concentric)','downhill (eccentric)'}, ...
       'Location','northeast');

fprintf('At l_CE = l_opt, |v_CE| = 0.3 v_max:\n');
fprintf('  uphill   (shortening): M = %.3f -> activation needed for F: A = F/(Fmax*%.3f)\n', Mup, Mup);
fprintf('  downhill (lengthening): M = %.3f -> activation needed for F: A = F/(Fmax*%.3f)\n', Mdown, Mdown);
fprintf('  ratio of required activation (up/down) = %.1f\n', Mdown/Mup);

%% ---------- Local functions ----------
function fl = f_length(lCE, mp)
% Active force-length relation (bell-shaped, no passive elasticity)
%   f_l = exp( ln(c) * | (lCE - lopt) / (w*lopt) |^3 )
fl = exp(log(mp.c) .* abs((lCE - mp.lopt) ./ (mp.w * mp.lopt)).^3);
end

function fv = f_velocity(vCE, mp)
% Hill-type force-velocity relation
%   concentric (v <= 0): f_v = (vmax + v) / (vmax - K v)
%   eccentric  (v >  0): f_v = N + (N-1) (v - vmax) / (7.56 K v + vmax)
fv = zeros(size(vCE));
con = vCE <= 0;
ecc = ~con;
fv(con) = (mp.vmax + vCE(con)) ./ (mp.vmax - mp.K .* vCE(con));
fv(ecc) = mp.N + (mp.N - 1) .* (vCE(ecc) - mp.vmax) ./ (7.56 * mp.K .* vCE(ecc) + mp.vmax);
end
