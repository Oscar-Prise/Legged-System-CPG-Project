function [t, lm, Fm, vm, A] = run_quick_release(m, tq, Tsim)
% RUN_QUICK_RELEASE  Simulate the quick-release experiment of Fig. 1.
%
%   [t, lm, Fm, vm, A] = run_quick_release(m, tq, Tsim)
%
%   A CE (build_CE.m, lm = lCE) hangs from the ceiling with a mass m below.
%   For t < tq the muscle is locked at lopt and fully stimulated (S = 1);
%   at t = tq the lock is released and the muscle pulls against the mass:
%
%       m * d2lm/dt2 = m*g - Fm          (lm measured downward, Fig. 1)
%
%   Returns time, muscle length [m], muscle force [N], muscle velocity [m/s]
%   (vm > 0 lengthening) and activation.
%   Reads the parameter struct "mp" from the base workspace (see p1_4_quick_release.m).

mdl = 'p1_4_quick_release';
if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist([mdl '.slx'], 'file'), delete([mdl '.slx']); end
new_system(mdl); open_system(mdl);

% Parameters used by the blocks (base workspace)
assignin('base', 'm_load', m);
assignin('base', 'tq', tq);

% --- muscle ------------------------------------------------------------
add_block('simulink/Sources/Constant', [mdl '/S = 1'], 'Value', '1', 'Position', [40 40 70 70]);
build_CE([mdl '/CE']);
set_param([mdl '/CE'], 'Position', [160 30 280 170]);

% --- mass dynamics:  acc = g - Fm/m  (only after release) --------------
add_block('simulink/Math Operations/Gain', [mdl '/1 over m'], 'Gain', '1/m_load', 'Position', [340 40 380 70]);
add_block('simulink/Sources/Constant', [mdl '/g'], 'Value', '9.81', 'Position', [340 100 380 130]);
add_block('simulink/Math Operations/Sum', [mdl '/g - F over m'], 'Inputs', '-+', 'Position', [430 60 460 90]);
add_block('simulink/Sources/Clock', [mdl '/Clock'], 'Position', [430 150 460 180]);
add_block('simulink/Sources/Constant', [mdl '/zero'], 'Value', '0', 'Position', [430 220 460 250]);
add_block('simulink/Signal Routing/Switch', [mdl '/lock'], 'Criteria', 'u2 > Threshold', ...
    'Threshold', 'tq', 'Position', [520 120 560 200]);
add_block('simulink/Continuous/Integrator', [mdl '/vm'], 'InitialCondition', '0', 'Position', [620 145 650 175]);
add_block('simulink/Continuous/Integrator', [mdl '/lm'], 'InitialCondition', 'mp.lopt', 'Position', [700 145 730 175]);

add_line(mdl, 'S = 1/1',        'CE/1',            'autorouting', 'on');
add_line(mdl, 'CE/1',           '1 over m/1',      'autorouting', 'on');
add_line(mdl, '1 over m/1',     'g - F over m/1',  'autorouting', 'on');
add_line(mdl, 'g/1',            'g - F over m/2',  'autorouting', 'on');
add_line(mdl, 'g - F over m/1', 'lock/1',          'autorouting', 'on');  % after release
add_line(mdl, 'Clock/1',        'lock/2',          'autorouting', 'on');
add_line(mdl, 'zero/1',         'lock/3',          'autorouting', 'on');  % locked: acc = 0
add_line(mdl, 'lock/1',         'vm/1',            'autorouting', 'on');
add_line(mdl, 'vm/1',           'lm/1',            'autorouting', 'on');
add_line(mdl, 'lm/1',           'CE/2',            'autorouting', 'on');  % lCE = lm
add_line(mdl, 'vm/1',           'CE/3',            'autorouting', 'on');  % vCE = vm

% --- logging -------------------------------------------------------------
logs = {'Fm_log', 'CE/1'; 'A_log', 'CE/2'; 'vm_log', 'vm/1'; 'lm_log', 'lm/1'};
for k = 1:size(logs, 1)
    add_block('simulink/Sinks/To Workspace', [mdl '/' logs{k,1}], 'VariableName', logs{k,1}, ...
        'SaveFormat', 'Timeseries', 'Position', [800 40+60*k 860 60+60*k]);
    add_line(mdl, logs{k,2}, [logs{k,1} '/1'], 'autorouting', 'on');
end

set_param(mdl, 'StopTime', num2str(Tsim), 'Solver', 'ode45', 'MaxStep', '2e-4', ...
    'RelTol', '1e-7', 'ReturnWorkspaceOutputs', 'on');
save_system(mdl);

out = sim(mdl);
t  = out.lm_log.Time;
lm = squeeze(out.lm_log.Data);
Fm = squeeze(out.Fm_log.Data);
vm = squeeze(out.vm_log.Data);
A  = squeeze(out.A_log.Data);
end
