%% Stage 1b - Passive paired SLIPs (Lanini et al. 2017, Sec. 3.2, Eqs. 2-6)
% Two spring-mass walkers (walker 1 behind, walker 2 in front) hold an object of
% mass mt through horizontal spring-damper arms.  Nothing adds energy, so the arm
% dampers slowly drain the pair (Lanini Sec. 3.2: only approximately periodic gaits).
%
% Outputs (the Stage 1 milestone of the proposal):
%   - does the pair synchronize, and at which phase lag (pace / trot / other)?
%   - energy dissipated by the arms per step  -> what a CPG push-off must put back
%   - energy balance check:  E_mech(t) + E_diss(t) = E_mech(0)
%
% Run from the folder that holds the slip_*.m files.

clear; close all; clc;

%% Settings -------------------------------------------------------------------
set_name  = 'proposal';  % 'proposal' (Geyer 2006 Fig. 3A walker) or 'lanini' (Lanini Table 6)
dE        = 5;           % [J] walker 1 gets E - dE, walker 2 gets E + dE (Lanini Table 5, Step 2: 810/840 J)
start_lbl = 'pace';      % 'pace': both start on leg 1;  'trot': walker 2 starts on leg 2
dt_offset = 0;           % [s] walker 2 walks alone this long before coupling (sets an initial phase offset)
tEnd      = 30;          % [s] Lanini used 100 s
D         = 2.0;         % [m] initial distance walker 1 -> walker 2 (drops out of the dynamics)
animate   = false;

%% Initial conditions ----------------------------------------------------------
P = slip_params(set_name);
sf = slip_find_fixed_point(P, [0; P.l0 - 0.025], P.E);   % single-walker fixed point (Stage 1a)
if any(isnan(sf)), error('no single-walker fixed point for this parameter set'); end
fprintf('[%s] single-walker fixed point: dx* = %.4f m, y* = %.4f m\n', set_name, sf(1), sf(2));

W1 = slip_walker_at_apex(P, sf(1), sf(2), P.E - dE, 0);
W2 = slip_walker_at_apex(P, sf(1), sf(2), P.E + dE, 0);
if dt_offset > 0                                   % let walker 2 run ahead in its own cycle
    o2 = slip_sim(P, W2, struct('tEnd', dt_offset));
    if ~strcmp(o2.status, 'ok'), error('walker 2 fell during the offset run'); end
    yl = o2.seg{end}.y(end, :);
    W2 = o2.W;  W2.x = yl(1);  W2.z = yl(2);  W2.xd = yl(3);  W2.zd = yl(4);
    I1 = slip_idx(1);  W2.phi = yl(I1.phi);
end
if strcmp(start_lbl, 'trot')                       % same mechanics, swapped leg labels
    W2.contact = fliplr(W2.contact);  W2.foot = fliplr(W2.foot);  W2.phi = W2.phi + pi;
end
shift = W1.x + D - W2.x;                           % put walker 2 a distance D ahead
W2.x = W2.x + shift;  W2.foot = W2.foot + shift;
W = [W1, W2];

%% Simulate -------------------------------------------------------------------
tic;
out = slip_sim(P, W, struct('tEnd', tEnd));
fprintf('simulated %.1f s in %.1f s CPU, status: %s\n', out.seg{end}.t(end), toc, out.status);
r = slip_post(out);
g = slip_gait_metrics(out, r);

%% Report ---------------------------------------------------------------------
Ebal = max(abs(r.Ebal));
fprintf('energy balance error  max|E(t) + Ediss(t) - E(0)| = %.2e J\n', Ebal);
fprintf('steps: walker 1 %d, walker 2 %d\n', numel(g.tTD{1}), numel(g.tTD{2}));
fprintf('total energy dissipated by the arms: %.2f J\n', r.Ediss(end));
nl = min(10, numel(g.Ediss_step));
if nl > 0
    fprintf('arm loss per step (mean of last %d steps): %.4f J  (%.2e of the pair''s energy)\n', ...
            nl, mean(g.Ediss_step(end-nl+1:end)), mean(g.Ediss_step(end-nl+1:end)) / r.Etot(1));
end
if ~isempty(g.phi)
    fprintf('stride phase lag at the end: %.1f deg  (0 = pace, 180 = trot, ~225 = diagonal sequence)\n', g.phi(end));
end
if numel(g.Tstep{1}) > 2 && numel(g.Tstep{2}) > 2
    fprintf('final step periods: %.4f s / %.4f s\n', g.Tstep{1}(end), g.Tstep{2}(end));
end
if numel(g.dApex{1}) > 1
    fprintf('last apex-to-apex change |d(dx,y)|: %.1e / %.1e m  (Lanini tolerance 1e-3)\n', ...
            g.dApex{1}(end), g.dApex{2}(end));
end

%% Plots ----------------------------------------------------------------------
figure('Name', 'Paired SLIPs', 'Position', [40 40 1300 720]);
subplot(2, 3, 1); hold on; grid on;
plot(r.t, r.L - out.L0, 'LineWidth', 1.5);
xlabel('time [s]'); ylabel('L - L_0 [m]'); legend({'arm 1 (rear)', 'arm 2 (front)'}); title('Arm stretch');
subplot(2, 3, 2); hold on; grid on;
plot(r.t, r.xd, 'LineWidth', 1.2);
xlabel('time [s]'); ylabel('dx/dt [m/s]'); legend({'walker 1', 'walker 2'}); title('Forward speed');
subplot(2, 3, 3); hold on; grid on;
m = r.t > r.t(end) - 3;
plot(r.t(m), r.z(m, 1), 'LineWidth', 2); plot(r.t(m), r.z(m, 2), '--', 'LineWidth', 2);
xlabel('time [s]'); ylabel('y [m]'); title('CoM height, last 3 s');
subplot(2, 3, 4); hold on; grid on;
plot(r.t, r.Ediss, 'LineWidth', 2);
xlabel('time [s]'); ylabel('E_{diss} [J]'); title(sprintf('Arm dissipation (balance err %.0e J)', Ebal));
subplot(2, 3, 5); hold on; grid on;
if ~isempty(g.Ediss_step), semilogy(g.tStep, max(g.Ediss_step, 1e-12), 'o-'); end
set(gca, 'YScale', 'log'); xlabel('time [s]'); ylabel('J per step'); title('Arm loss per step');
subplot(2, 3, 6); hold on; grid on;
if ~isempty(g.phi), plot(g.tPhi, g.phi, 'o-', 'LineWidth', 1.5); end
yline(0, ':'); yline(180, ':'); yline(225, ':'); ylim([-10 370]);
xlabel('time [s]'); ylabel('\phi [deg]'); title('Stride phase lag, walker 2 vs 1');

if animate, slip_animate(out, r, 1); end
save(sprintf('stage1b_%s_dE%g_%s.mat', set_name, dE, start_lbl), 'out', 'g', 'P');
