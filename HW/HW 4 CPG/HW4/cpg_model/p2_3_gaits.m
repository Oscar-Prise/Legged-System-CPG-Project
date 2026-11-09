%% Problem 2.3 - Two coupled half-center oscillators: walking, running, hopping
% Four neurons (eL, fL, eR, fR) from build_neuron.m, connected all-to-all as
% in Fig. 2 of the assignment through a 4x4 weight matrix W, where
%     W(i,j) = weight from neuron j INTO neuron i,   order [eL fL eR fR].
% Each weight is a Gain block on the wire; the neuron blocks never change.
% Three weight presets reproduce the extensor patterns of Fig. 2.
% Run from the cpg_model folder.

clear; close all; clc;

%% Neuron parameters
np.x0 = 5;  np.taux = 1;  np.taua = 12;  np.wp = 2.5;
Tsim  = 200;

names = {'eL', 'fL', 'eR', 'fR'};
idx   = struct('eL', 1, 'fL', 2, 'eR', 3, 'fR', 4);

%% Build the 4-neuron network once; weights are read from the matrix W
mdl = 'p2_3_gaits';
if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist([mdl '.slx'], 'file'), delete([mdl '.slx']); end
new_system(mdl); open_system(mdl);

add_block('simulink/Sources/Constant', [mdl '/x0'], 'Value', 'np.x0', 'Position', [20 300 60 330]);
ypos = [40 200 400 560];
for i = 1:4
    build_neuron([mdl '/' names{i}], sprintf('xinit(%d)', i));          % initial potential from vector xinit
    set_param([mdl '/' names{i}], 'Position', [260 ypos(i) 360 ypos(i)+90]);
    add_line(mdl, 'x0/1', [names{i} '/1'], 'autorouting', 'on');       % same tonic drive to all
    % the input sum_i w_i y_i of neuron i: Sum of the three other outputs, each through a Gain W(i,j)
    add_block('simulink/Math Operations/Sum', [mdl '/sum ' names{i}], 'Inputs', '+++', ...
        'Position', [180 ypos(i)+40 200 ypos(i)+70]);
    add_line(mdl, ['sum ' names{i} '/1'], [names{i} '/2'], 'autorouting', 'on');
    add_block('simulink/Sinks/To Workspace', [mdl '/' names{i} '_log'], 'VariableName', ['y_' names{i}], ...
        'SaveFormat', 'Timeseries', 'Position', [460 ypos(i)+30 520 ypos(i)+50]);
    add_line(mdl, [names{i} '/1'], [names{i} '_log/1'], 'autorouting', 'on');
end
for i = 1:4                                   % wire  y_j --> Gain W(i,j) --> sum_i
    others = setdiff(1:4, i);
    for k = 1:3
        j = others(k);
        g = sprintf('W_%s_from_%s', names{i}, names{j});
        add_block('simulink/Math Operations/Gain', [mdl '/' g], 'Gain', sprintf('W(%d,%d)', i, j), ...
            'Orientation', 'left', 'Position', [100 ypos(i)+25*k 140 ypos(i)+25*k+20]);
        add_line(mdl, [names{j} '/1'], [g '/1'], 'autorouting', 'on');
        add_line(mdl, [g '/1'], sprintf('sum %s/%d', names{i}, k), 'autorouting', 'on');
    end
end
set_param(mdl, 'StopTime', num2str(Tsim), 'Solver', 'ode45', 'MaxStep', '0.02', ...
    'RelTol', '1e-6', 'ReturnWorkspaceOutputs', 'on');
save_system(mdl);

%% Gait presets  (w_ef: e->f within leg, w_fe: f->e within leg,
%                 w_ee: eL<->eR, w_ff: fL<->fR, w_ef_x: e -> contralateral f)
gaits(1) = struct('name', 'Walk', 'w_ef', -2.0, 'w_fe', -1.0, 'w_ee', -0.5, 'w_ff', -0.5, 'w_ef_x', 0,    'xinit', [1 0 0 0]);
gaits(2) = struct('name', 'Run',  'w_ef', -1.0, 'w_fe', -2.5, 'w_ee', -1.0, 'w_ff',  0,   'w_ef_x', 0,    'xinit', [1 0 0 0]);
gaits(3) = struct('name', 'Hop',  'w_ef', -1.0, 'w_fe', -2.5, 'w_ee',  0,   'w_ff',  0,   'w_ef_x', -0.5, 'xinit', [1 0 0 0]);

figure('Name', 'Gaits', 'Position', [60 60 1300 600]);
for g = 1:3
    G = gaits(g);
    W = zeros(4);
    W(idx.fL, idx.eL) = G.w_ef;   W(idx.eL, idx.fL) = G.w_fe;      % left leg half-center
    W(idx.fR, idx.eR) = G.w_ef;   W(idx.eR, idx.fR) = G.w_fe;      % right leg half-center
    W(idx.eR, idx.eL) = G.w_ee;   W(idx.eL, idx.eR) = G.w_ee;      % extensor <-> extensor
    W(idx.fR, idx.fL) = G.w_ff;   W(idx.fL, idx.fR) = G.w_ff;      % flexor <-> flexor
    W(idx.fR, idx.eL) = G.w_ef_x; W(idx.fL, idx.eR) = G.w_ef_x;    % extensor -> contralateral flexor
    xinit = G.xinit;                                                %#ok<NASGU>  read by the integrators

    out = sim(mdl);
    t = out.y_eL.Time;
    y = [squeeze(out.y_eL.Data) squeeze(out.y_fL.Data) squeeze(out.y_eR.Data) squeeze(out.y_fR.Data)];

    r = gait_stats(t, y, 100);
    fprintf('%-4s: period %.1f s | extensor duty %.2f | eR-eL phase %.2f | both ext off %.2f | both ext on %.2f\n', ...
        G.name, r.period, r.duty, r.phase, r.both_off, r.both_on);

    % plot three periods after the transient, traces on top and Fig.2-style bars below
    T0 = 120;  m = t >= T0 & t <= T0 + 3*r.period;  tt = t(m) - T0;
    subplot(2,3,g); hold on; grid on;
    plot(tt, y(m,3), 'r', 'LineWidth', 3);  plot(tt, y(m,1), 'b', 'LineWidth', 1.8);
    plot(tt, y(m,2), 'b:', tt, y(m,4), 'r:');
    ylim([0 4.2]); xlim([0 tt(end)]); ylabel('output');
    title(sprintf('%s: w_{ef}=%g, w_{fe}=%g, w_{ee}=%g, w_{ff}=%g, w_{ef,x}=%g', G.name, G.w_ef, G.w_fe, G.w_ee, G.w_ff, G.w_ef_x), 'FontSize', 9);
    if g == 1, legend({'y_{eR}', 'y_{eL}', 'y_{fL}', 'y_{fR}'}, 'Location', 'northeast'); end
    subplot(2,3,g+3); hold on; grid on;
    draw_bars(tt, y(m,3) > 1e-6, 1);  draw_bars(tt, y(m,1) > 1e-6, 0);
    yticks([0 1]); yticklabels({'eL', 'eR'}); ylim([-0.6 1.6]); xlim([0 tt(end)]); xlabel('time [s]');
end
if ~exist('figures', 'dir'), mkdir('figures'); end
exportgraphics(gcf, 'figures/p2_3_gaits.png', 'Resolution', 300);

%% ---- local functions ---------------------------------------------------
function r = gait_stats(t, y, t0)
% period and duty from eL onsets; phase of eR onset relative to eL; overlap measures
m = t > t0;  t = t(m);  eL = y(m,1);  eR = y(m,3);
onL = eL > 1e-6;  onR = eR > 1e-6;
oL = find(diff(onL) == 1);  oR = find(diff(onR) == 1);
r.period = mean(diff(t(oL)));  r.duty = mean(onL(oL(1):oL(end)));
tL = t(oL(end-1));  tR = t(oR(find(t(oR) >= tL, 1)));
r.phase = mod((tR - tL) / r.period, 1);
r.both_off = mean(~onL & ~onR);  r.both_on = mean(onL & onR);
end

function draw_bars(t, on, level)
d = diff([0; on(:); 0]);  s = find(d == 1);  e = find(d == -1) - 1;
for k = 1:numel(s), plot([t(s(k)) t(e(k))], [level level], 'k', 'LineWidth', 6); end
end
