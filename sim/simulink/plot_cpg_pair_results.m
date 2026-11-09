%% plot_cpg_pair_results.m - plots after running cpg_pair.slx
% Uses the logged signals in 'out' (Simulink run) and mp from the workspace.

if ~exist('out', 'var'), error('no ''out'' in the workspace: run the model first'); end
ts = @(name) out.get(name);
P  = ts('pos_log');  V = ts('vel_log');  PH = ts('phi_log');  TD = ts('td_log');  EL = ts('Eloss_log');
t   = P.Time;  pos = squeeze(P.Data);  vel = squeeze(V.Data);  phi = squeeze(PH.Data);
if size(pos, 1) ~= numel(t), pos = pos.'; vel = vel.'; phi = phi.'; end
tdd = squeeze(TD.Data);  if size(tdd, 1) ~= numel(TD.Time), tdd = tdd.'; end
Eloss = squeeze(EL.Data);  if size(Eloss, 1) ~= numel(EL.Time), Eloss = Eloss.'; end
nW = mp.nW;

% touchdown times of leg 1 of each walker
tTD1 = cell(1, nW);  tTDall = cell(1, nW);
for i = 1:nW
    tTD1{i}   = TD.Time(tdd(:, i) == 1);
    tTDall{i} = TD.Time(tdd(:, i) > 0);
end

fprintf('\nsimulated %.2f s', t(end));
if t(end) < tmax - 1e-6, fprintf('  -> stopped early (fall)'); end
fprintf('\n');
for i = 1:nW
    Ti = diff(tTDall{i});
    if numel(Ti) > 4
        v = (interp1(t, pos(:, 2*i-1), tTDall{i}(end)) - interp1(t, pos(:, 2*i-1), tTDall{i}(end-4))) ...
            / (tTDall{i}(end) - tTDall{i}(end-4));
        fprintf('walker %d: %d steps, last step %.4f s, speed over last 4 steps %.3f m/s\n', i, numel(tTDall{i}), Ti(end), v);
    end
end
fprintf('energy lost: arms %.3f J, leg dampers %.2f J\n', Eloss(end, 1), Eloss(end, 2));

% stride phase lag, homologous legs (0 = pace, 180 = trot, ~225 = diagonal sequence; Lanini 2017)
lag = [];  tl = [];
if nW == 2
    a = tTD1{1};  b = tTD1{2};
    for n = 1:numel(a) - 1
        [~, m] = min(abs(b - a(n)));
        if isempty(m), continue; end
        lag(end+1) = mod(360*(b(m) - a(n))/(a(n+1) - a(n)), 360);   %#ok<SAGROW>
        tl(end+1) = a(n);                                           %#ok<SAGROW>
    end
    if ~isempty(lag), fprintf('stride phase lag at the end: %.1f deg\n', lag(end)); end
end

figure('Name', 'cpg_pair results', 'Position', [60 60 1200 620]);
subplot(2, 3, 1); plot(t, vel(:, 1:2:2*nW), 'LineWidth', 1.2); grid on;
xlabel('time [s]'); ylabel('dx/dt [m/s]'); title('Forward speed');
subplot(2, 3, 2); plot(t, pos(:, 2:2:2*nW), 'LineWidth', 1.2); grid on; xlim(max(t(end) - 3, 0) + [0 3]);
xlabel('time [s]'); ylabel('z [m]'); title('CoM height (last 3 s)');
subplot(2, 3, 3); plot(EL.Time, Eloss, 'LineWidth', 1.5); grid on;
xlabel('time [s]'); ylabel('[J]'); legend({'arm dampers', 'leg dampers'}, 'Location', 'northwest'); title('Energy lost');
subplot(2, 3, 4); plot(t, phi(:, 1:nW), '.'); grid on;
xlabel('time [s]'); ylabel('\phi [rad]'); title('CPG phase (resets at touchdown)');
subplot(2, 3, 5);
if nW == 2
    plot(t, [pos(:, 5) - pos(:, 1) - mp.L0(1), pos(:, 3) - pos(:, 5) - mp.L0(2)], 'LineWidth', 1.2); grid on;
    legend({'rear arm', 'front arm'}); xlabel('time [s]'); ylabel('L - L_0 [m]'); title('Arm stretch');
end
subplot(2, 3, 6);
if ~isempty(lag)
    plot(tl, lag, 'o-'); grid on; ylim([-10 370]); yline(180, ':'); yline(225, ':');
    xlabel('time [s]'); ylabel('\phi_{lag} [deg]'); title('Stride phase lag, walker 2 vs 1');
end
