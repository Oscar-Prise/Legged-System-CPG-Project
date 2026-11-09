%% Assignment 2 result plotting
% Expected workspace variables:
% time, CoM, vCoM, RFP, LFP, Rstance_log, Lstance_log, mp

t = time(:);

com = CoM(:,1:3);
rfp = RFP(:,1:3);
lfp = LFP(:,1:3);

rs = Rstance_log(:) > 0.5;
ls = Lstance_log(:) > 0.5;

%% Find touchdown events
Rtouch = find(rs & [true; ~rs(1:end-1)]);
Ltouch = find(ls & [true; ~ls(1:end-1)]);

%% Footfall + CoM plot
figure;
hold on;

plot(com(:,1), com(:,2), 'k-', 'LineWidth', 1.5);
scatter(rfp(Rtouch,1), rfp(Rtouch,2), 50, 'r', 'filled');
scatter(lfp(Ltouch,1), lfp(Ltouch,2), 50, 'b', 'filled');

axis equal;
grid on;
xlabel('x [m]');
ylabel('y [m]');
legend('CoM trajectory', 'Right footsteps', 'Left footsteps', ...
       'Location', 'best');
title('3-D LIPM Walking Pattern');

%% Leg lengths

Lright = sqrt(sum((com - rfp).^2, 2));
Lleft  = sqrt(sum((com - lfp).^2, 2));

% Only evaluate a leg while it is in stance
Lright_stance = Lright(rs);
Lleft_stance  = Lleft(ls);

Lmax_sim = max([Lright_stance; Lleft_stance]);

fprintf('Maximum stance-leg length = %.4f m\n', Lmax_sim);
fprintf('Allowed leg length        = %.4f m\n', mp.Lmax);

%% Walking-area dimensions
span_x = max(com(:,1)) - min(com(:,1));
span_y = max(com(:,2)) - min(com(:,2));

fprintf('CoM x span = %.3f m\n', span_x);
fprintf('CoM y span = %.3f m\n', span_y);

%% Speed
speed = sqrt(sum(vCoM(:,1:2).^2,2));

figure;
plot(t, speed, 'LineWidth', 1.25);
grid on;
xlabel('Time [s]');
ylabel('CoM speed [m/s]');
title('CoM Speed');
