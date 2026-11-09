%% Stage 2 - One CPG walker: does CPG-timed push-off give a limit cycle? (H1)
%
% Walker = Stage 1 spring-mass walker (massless legs, touchdown at fixed angle a0) plus
%   - a leg damper  F = k (lambda - l) - c dl/dt          (energy sink; team decision 2026-11-09)
%   - a push-off: rest length lambda = l0 + dl u(theta) in late stance
%   - one phase oscillator per walker  (Owaki & Ishiguro 2017, Eq. 1)
%         dphi/dt = omega - sigma sum_j N_j cos(theta_j),   theta_j = phi + (j-1) pi
%     reset at touchdown of leg j:  theta_j <- th_TD = 2 pi (1 - beta)   (Aoi & Tsuchiya 2007)
%
% H1 test: find the fixed point of the apex map  (dx, y, xd, th)_i -> (dx, y, xd, th)_{i+1}
% and check that all eigenvalues are inside the unit circle; then kick the walker and
% watch it return.  Energy check: E(t) + leg loss + arm loss - actuator work = E(0).
%
% Run from the folder that holds the slip_*.m files.

clear; close all; clc;

%% Settings -------------------------------------------------------------------
P = slip_params('proposal');
P.cpg   = true;
P.c     = 100;          % [Ns/m]  leg damper
P.dl    = 0.01;         % [m]     push-off leg extension
P.omega = 2*pi/1.0;     % [rad/s] one CPG cycle = one stride; a bit slower than the passive stride
                        %         (~0.89 s, Stage 1a) so each touchdown reset moves the phase forward
sigmas  = [0 3];        % [rad/s per body weight] load feedback gains to test (0 = pure timer)
kick    = 0.10;         % relative forward-speed kick for the perturbation test

%% Fixed point (limit cycle) for each feedback gain ------------------------------
R = struct([]);
for s = 1:numel(sigmas)
    P.sigma = sigmas(s);
    % walk 40 steps from the passive fixed point to land near the cycle
    W = slip_walker_at_apex(P, 0, 0.9708, P.E);
    o = slip_sim(P, W, struct('tEnd', 60, 'nApexStop', 40));
    if ~strcmp(o.status, 'ok'), fprintf('sigma = %g: walker fell (%s)\n', P.sigma, o.status); continue; end
    a = o.ev(strcmp({o.ev.type}, 'APEX'));  a = a(end);  I1 = slip_idx(1);
    s0 = [a.y(1) - a.foot(a.leg); a.y(2); a.y(3); mod(a.y(I1.phi) + (a.leg - 1)*pi, 2*pi)];

    [sf, lam, info] = slip_find_fixed_point(P, s0);
    if ~info.ok, fprintf('sigma = %g: Newton did not converge\n', P.sigma); continue; end
    if all(abs(lam) < 1), verdict = 'STABLE limit cycle (H1 holds here)'; else, verdict = 'unstable'; end
    fprintf('\nsigma = %g rad/s/bw, c = %g Ns/m, dl = %.3f m, omega = %.2f rad/s\n', P.sigma, P.c, P.dl, P.omega);
    fprintf('  fixed point: dx* = %+.4f m, y* = %.4f m, xd* = %.4f m/s, th* = %.3f rad\n', sf);
    fprintf('  speed %.3f m/s, step time %.3f s\n', info.v, info.T);
    fprintf('  |eigenvalues| = %s  -> %s\n', mat2str(sort(abs(lam)).', 3), verdict);

    % one steady stride for the energy budget and plots
    W = slip_walker_at_apex(P, sf(1), sf(2), 1e4);  W.xd = sf(3);  W.phi = sf(4);
    o = slip_sim(P, W, struct('tEnd', 10, 'nApexStop', 2));     % exactly two steps, apex to apex
    r = slip_post(o);
    fprintf('  per step: push-off work %.2f J = leg loss %.2f J (balance residual %.1e J)\n', ...
            r.Wact(end)/2, r.Eleg(end)/2, max(abs(r.Ebal)));

    % perturbation test: kick the forward speed, count steps until back on the cycle
    W = slip_walker_at_apex(P, sf(1), sf(2), 1e4);  W.xd = sf(3)*(1 + kick);  W.phi = sf(4);
    op = slip_sim(P, W, struct('tEnd', 60, 'nApexStop', 40));
    ap = op.ev(strcmp({op.ev.type}, 'APEX'));
    vap = arrayfun(@(e) e.y(3), ap);
    fprintf('  after a %+.0f%% speed kick: %s, apex speed error after 40 steps %.1e m/s\n', ...
            100*kick, op.status, abs(vap(end) - sf(3)));

    R(end+1).sigma = P.sigma;  R(end).sf = sf;  R(end).lam = lam;  R(end).info = info;   %#ok<SAGROW>
    R(end).r = r;  R(end).vap = vap;  R(end).P = P;
end
if isempty(R), return; end

%% Plots ----------------------------------------------------------------------
figure('Name', 'CPG walker', 'Position', [40 40 1300 700]);
for s = 1:numel(R)
    r = R(s).r;  Pp = R(s).P;  lbl = sprintf('\\sigma = %g', R(s).sigma);
    subplot(2, 3, 1); hold on; grid on;
    plot(r.t, r.FzLeg{1}(:, 1)/(Pp.m*Pp.g), 'LineWidth', 2, 'DisplayName', ['leg 1, ' lbl]);
    xlabel('time [s]'); ylabel('vertical GRF [bw]'); title('Leg-1 force over one stride'); legend('Location', 'southwest');
    subplot(2, 3, 2); hold on; grid on;
    th1 = mod(r.phi, 2*pi);
    plot(r.t, th1, '.', 'DisplayName', lbl);
    xlabel('time [s]'); ylabel('\theta_1 [rad]'); title('CPG phase of leg 1 (resets at TD)'); legend('Location', 'southwest');
    subplot(2, 3, 3); hold on; grid on;
    u = arrayfun(@(th) slip_pushoff(th, Pp), r.phi);
    plot(r.t, Pp.l0 + Pp.dl*u, 'LineWidth', 2, 'DisplayName', lbl);
    xlabel('time [s]'); ylabel('\lambda_1 [m]'); title('Leg-1 rest length (push-off)'); legend('Location', 'southwest');
    subplot(2, 3, 4); hold on; grid on;
    plot(r.t, r.Wact, 'LineWidth', 2, 'DisplayName', ['actuator work, ' lbl]);
    plot(r.t, r.Eleg, '--', 'LineWidth', 2, 'DisplayName', ['leg loss, ' lbl]);
    xlabel('time [s]'); ylabel('[J]'); title('Energy in = energy out over a stride'); legend('Location', 'southwest');
    subplot(2, 3, 5); hold on; grid on;
    plot(1:numel(R(s).vap), R(s).vap, 'o-', 'DisplayName', lbl);
    xlabel('step'); ylabel('apex speed [m/s]'); title(sprintf('Recovery from a %+.0f%% speed kick', 100*kick)); legend('Location', 'southwest');
    subplot(2, 3, 6); hold on; grid on;
    th = linspace(0, 2*pi, 200);
    plot(cos(th), sin(th), 'k:');
    plot(real(R(s).lam), imag(R(s).lam), 'x', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', lbl);
    axis equal; xlabel('Re'); ylabel('Im'); title('Return-map eigenvalues');
end
save('stage2_cpg_walker.mat', 'R');
