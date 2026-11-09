%% Stage 1a - One passive spring-mass walker (Geyer, Seyfarth & Blickhan 2006)
% Finds the period-1 walking fixed point of the apex return map
%     R: (dx, y)_i -> (dx, y)_{i+1}       (Geyer 2006, App. A(b))
% checks its stability (eigenvalues of dR/ds inside the unit circle), and plots one
% steady step: CoM height and the ground reaction forces (expect the M-shaped
% vertical GRF of Geyer 2006 Fig. 3).
%
% This walker is the building block of the Lanini pair (Stage 1b) and of the CPG
% walker (Stage 2).  Run from the folder that holds the slip_*.m files.

clear; close all; clc;

sets = {'proposal', 'lanini'};
FP = struct();
for s = 1:numel(sets)
    P = slip_params(sets{s});

    % scan apex heights between landing height and rest length (Geyer 2006 App. A(c)):
    % iterate the map from each start, polish survivors with Newton, keep distinct fixed points
    ys = linspace(P.l0*sin(P.a0) + 0.005, P.l0 - 0.002, 12);
    fps = zeros(2, 0);  lams = zeros(2, 0);  infos = {};
    for y = ys
        sk = [0; y];  ok = true;
        for n = 1:30
            [sk, ok] = slip_apex_map(P, sk, P.E);
            if ~ok, break; end
        end
        if ~ok, continue; end
        [sn, ln, in] = slip_find_fixed_point(P, sk, P.E);
        if in.ok && (isempty(fps) || min(sqrt(sum((fps - sn).^2, 1))) > 1e-5)
            fps(:, end+1) = sn;  lams(:, end+1) = ln;  infos{end+1} = in;          %#ok<SAGROW>
        end
    end
    if isempty(fps), fprintf('%s: no fixed point found at E = %g J\n', sets{s}, P.E); continue; end
    fprintf('\n[%s] fixed points found:\n', sets{s});
    for c = 1:size(fps, 2)
        fprintf('   dx* = %+.4f m, y* = %.4f m, max|lambda| = %.3f\n', fps(1, c), fps(2, c), max(abs(lams(:, c))));
    end
    [~, c] = min(max(abs(lams), [], 1));            % continue with the most stable one
    sf = fps(:, c);  lam = lams(:, c);  info = infos{c};

    fprintf('\n[%s]  k = %.1f kN/m, alpha0 = %.1f deg, E = %g J\n', sets{s}, P.k/1e3, P.a0*180/pi, P.E);
    fprintf('  fixed point: dx* = %.4f m, y* = %.4f m  (y* - l0 = %.1f mm)\n', sf(1), sf(2), 1e3*(sf(2) - P.l0));
    fprintf('  speed %.3f m/s, step time %.3f s, step length %.3f m\n', info.v, info.T, info.v*info.T);
    if all(abs(lam) < 1), verdict = 'stable'; else, verdict = 'unstable'; end
    fprintf('  |eigenvalues| = %s  -> %s\n', mat2str(abs(lam.'), 3), verdict);
    FP.(sets{s}) = struct('sf', sf, 'lam', lam, 'info', info);

    % simulate 4 steps from the fixed point and plot the last two
    W = slip_walker_at_apex(P, sf(1), sf(2), P.E);
    o = slip_sim(P, W, struct('tEnd', 4*info.T + 0.01));
    r = slip_post(o);
    m = r.t >= 2*info.T;
    tt = r.t(m) - r.t(find(m, 1));
    figure('Name', ['Single walker - ' sets{s}], 'Position', [80 80 1000 380]);
    subplot(1, 2, 1); hold on; grid on;
    plot(tt, r.z(m), 'LineWidth', 2);
    yline(P.l0*sin(P.a0), ':k', 'TD height');
    xlabel('time [s]'); ylabel('CoM height y [m]');
    title(sprintf('%s: v = %.2f m/s, |\\lambda| = %.2f', sets{s}, info.v, max(abs(lam))));
    subplot(1, 2, 2); hold on; grid on;
    plot(tt, r.FzLeg{1}(m, 1)/(P.m*P.g), 'LineWidth', 2);
    plot(tt, r.FzLeg{1}(m, 2)/(P.m*P.g), 'LineWidth', 2);
    plot(tt, r.Fz(m)/(P.m*P.g), 'k:', 'LineWidth', 1);
    xlabel('time [s]'); ylabel('vertical GRF [body weights]'); legend({'leg 1', 'leg 2', 'sum'});
    title('Vertical GRF per leg (compare Geyer 2006 Fig. 3)');
    % conservative model: energy must stay constant
    fprintf('  energy drift over 4 steps: %.2e J\n', max(abs(r.Ebal)));
end
save('stage1a_fixed_points.mat', 'FP');
