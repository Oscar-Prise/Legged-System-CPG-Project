function [t, lm, Fm, vm, lce, A] = run_quick_release_SE(m, tq, Tsim)
% RUN_QUICK_RELEASE_SE  Quick-release experiment with a CE + SE muscle.
%
%   [t, lm, Fm, vm, lce, A] = run_quick_release_SE(m, tq, Tsim)
%
%   Same experiment as run_quick_release.m, but the muscle is the
%   muscle-tendon unit of build_MTU.m, so lm = lCE + lSE. The muscle is
%   locked at lm = lopt + lslack (SE just slack, CE at lopt) for t < tq.
%   Reads the parameter struct "mp" from the base workspace.

mdl = 'p1_6_quick_release_SE';
if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist([mdl '.slx'], 'file'), delete([mdl '.slx']); end
new_system(mdl); open_system(mdl);

assignin('base', 'm_load', m);
assignin('base', 'tq', tq);

% --- muscle-tendon unit ----------------------------------------------
add_block('simulink/Sources/Constant', [mdl '/S = 1'], 'Value', '1', 'Position', [40 40 70 70]);
build_MTU([mdl '/MTU']);
set_param([mdl '/MTU'], 'Position', [160 30 280 170]);

% --- mass dynamics:  acc = g - Fm/m  (only after release) --------------
add_block('simulink/Math Operations/Gain', [mdl '/1 over m'], 'Gain', '1/m_load', 'Position', [340 40 380 70]);
add_block('simulink/Sources/Constant', [mdl '/g'], 'Value', '9.81', 'Position', [340 100 380 130]);
add_block('simulink/Math Operations/Sum', [mdl '/g - F over m'], 'Inputs', '-+', 'Position', [430 60 460 90]);
add_block('simulink/Sources/Clock', [mdl '/Clock'], 'Position', [430 150 460 180]);
add_block('simulink/Sources/Constant', [mdl '/zero'], 'Value', '0', 'Position', [430 220 460 250]);
add_block('simulink/Signal Routing/Switch', [mdl '/lock'], 'Criteria', 'u2 > Threshold', ...
    'Threshold', 'tq', 'Position', [520 120 560 200]);
add_block('simulink/Continuous/Integrator', [mdl '/vm'], 'InitialCondition', '0', 'Position', [620 145 650 175]);
add_block('simulink/Continuous/Integrator', [mdl '/lm'], 'InitialCondition', 'mp.lopt + mp.lslack', 'Position', [700 145 730 175]);

add_line(mdl, 'S = 1/1',        'MTU/1',           'autorouting', 'on');
add_line(mdl, 'MTU/1',          '1 over m/1',      'autorouting', 'on');
add_line(mdl, '1 over m/1',     'g - F over m/1',  'autorouting', 'on');
add_line(mdl, 'g/1',            'g - F over m/2',  'autorouting', 'on');
add_line(mdl, 'g - F over m/1', 'lock/1',          'autorouting', 'on');
add_line(mdl, 'Clock/1',        'lock/2',          'autorouting', 'on');
add_line(mdl, 'zero/1',         'lock/3',          'autorouting', 'on');
add_line(mdl, 'lock/1',         'vm/1',            'autorouting', 'on');
add_line(mdl, 'vm/1',           'lm/1',            'autorouting', 'on');
add_line(mdl, 'lm/1',           'MTU/2',           'autorouting', 'on');   % lm = lCE + lSE

% --- logging -------------------------------------------------------------
logs = {'Fm_log', 'MTU/1'; 'lce_log', 'MTU/2'; 'A_log', 'MTU/4'; 'vm_log', 'vm/1'; 'lm_log', 'lm/1'};
for k = 1:size(logs, 1)
    add_block('simulink/Sinks/To Workspace', [mdl '/' logs{k,1}], 'VariableName', logs{k,1}, ...
        'SaveFormat', 'Timeseries', 'Position', [800 40+60*k 860 60+60*k]);
    add_line(mdl, logs{k,2}, [logs{k,1} '/1'], 'autorouting', 'on');
end

set_param(mdl, 'StopTime', num2str(Tsim), 'Solver', 'ode45', 'MaxStep', '1e-4', ...
    'RelTol', '1e-7', 'ReturnWorkspaceOutputs', 'on');
save_system(mdl);

out = sim(mdl);
t   = out.lm_log.Time;
lm  = squeeze(out.lm_log.Data);
Fm  = squeeze(out.Fm_log.Data);
vm  = squeeze(out.vm_log.Data);
lce = squeeze(out.lce_log.Data);
A   = squeeze(out.A_log.Data);
end
