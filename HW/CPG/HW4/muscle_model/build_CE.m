function build_CE(sys)
% BUILD_CE  Add a contractile-element (CE) subsystem to a Simulink model.
%
%   build_CE('model/CE') creates a subsystem at that path with
%     inputs : 1) S   [ ]    neural stimulation
%              2) lCE [m]    CE length
%              3) vCE [m/s]  CE velocity (<0 shortening, >0 lengthening)
%     outputs: 1) F_CE [N]   total CE force = Fmax * fl(lCE) * fv(vCE) * A
%              2) A    [ ]   muscle activation
%
%   Inside: the excitation-contraction coupling from Problem 1.2 and the
%   force-length / force-velocity relations from Problem 1.1, which are
%   evaluated with the lecture functions CE_force_length_relation.m and
%   CE_force_velocity_relation.m (must be on the path).
%
%   The block reads the parameter struct "mp" from the base workspace:
%     mp.Fmax, mp.lopt, mp.vmax, mp.w, mp.c, mp.N, mp.K, mp.tauR, mp.tauF, mp.A0

add_block('simulink/Ports & Subsystems/Subsystem', sys);
delete_line(sys, 'In1/1', 'Out1/1');
delete_block([sys '/In1']); delete_block([sys '/Out1']);

% ---- ports ---------------------------------------------------------
add_block('simulink/Sources/In1',  [sys '/S'],    'Port', '1', 'Position', [30  60  60  74]);
add_block('simulink/Sources/In1',  [sys '/lCE'],  'Port', '2', 'Position', [30 200  60 214]);
add_block('simulink/Sources/In1',  [sys '/vCE'],  'Port', '3', 'Position', [30 300  60 314]);
add_block('simulink/Sinks/Out1',   [sys '/F_CE'], 'Port', '1', 'Position', [720 160 750 174]);
add_block('simulink/Sinks/Out1',   [sys '/A'],    'Port', '2', 'Position', [720  60 750  74]);

% ---- excitation-contraction coupling (Problem 1.2) -----------------
add_block('simulink/Math Operations/Sum', [sys '/S - A'], 'Inputs', '+-', 'Position', [120 55 150 85]);
add_block('simulink/Sources/Constant', [sys '/1 over tauR'], 'Value', '1/mp.tauR', 'Position', [120 0 170 20]);
add_block('simulink/Sources/Constant', [sys '/1 over tauF'], 'Value', '1/mp.tauF', 'Position', [120 120 170 140]);
add_block('simulink/Signal Routing/Switch', [sys '/rising or falling'], ...
    'Criteria', 'u2 > Threshold', 'Threshold', '0', 'Position', [220 30 260 110]);
add_block('simulink/Math Operations/Product', [sys '/dA_dt'], 'Inputs', '2', 'Position', [310 50 340 110]);
add_block('simulink/Continuous/Integrator', [sys '/Integrator'], 'InitialCondition', 'mp.A0', 'Position', [390 65 420 95]);

add_line(sys, 'S/1',                 'S - A/1',             'autorouting', 'on');
add_line(sys, 'Integrator/1',        'S - A/2',             'autorouting', 'on');
add_line(sys, 'S - A/1',             'rising or falling/2', 'autorouting', 'on');
add_line(sys, 'S - A/1',             'dA_dt/1',             'autorouting', 'on');
add_line(sys, '1 over tauR/1',       'rising or falling/1', 'autorouting', 'on');
add_line(sys, '1 over tauF/1',       'rising or falling/3', 'autorouting', 'on');
add_line(sys, 'rising or falling/1', 'dA_dt/2',             'autorouting', 'on');
add_line(sys, 'dA_dt/1',             'Integrator/1',        'autorouting', 'on');
add_line(sys, 'Integrator/1',        'A/1',                 'autorouting', 'on');

% ---- force-length and force-velocity relations (Problem 1.1) -------
add_block('simulink/Math Operations/Gain', [sys '/1 over lopt'], 'Gain', '1/mp.lopt', 'Position', [120 192 160 222]);
add_block('simulink/Math Operations/Gain', [sys '/1 over vmax'], 'Gain', '1/mp.vmax', 'Position', [120 292 160 322]);
add_matlab_fcn([sys '/fl'], 'function fl = fcn(lce_nrm, mp)\nfl = CE_force_length_relation(lce_nrm, mp);', [220 185 300 229]);
add_matlab_fcn([sys '/fv'], 'function fv = fcn(vce_nrm, mp)\nfv = CE_force_velocity_relation(vce_nrm, mp);', [220 285 300 329]);

add_line(sys, 'lCE/1',         '1 over lopt/1', 'autorouting', 'on');
add_line(sys, 'vCE/1',         '1 over vmax/1', 'autorouting', 'on');
add_line(sys, '1 over lopt/1', 'fl/1',          'autorouting', 'on');
add_line(sys, '1 over vmax/1', 'fv/1',          'autorouting', 'on');

% ---- F_CE = Fmax * fl * fv * A -------------------------------------
add_block('simulink/Math Operations/Product', [sys '/A fl fv'], 'Inputs', '3', 'Position', [500 140 530 200]);
add_block('simulink/Math Operations/Gain', [sys '/Fmax'], 'Gain', 'mp.Fmax', 'Position', [600 152 640 182]);

add_line(sys, 'Integrator/1', 'A fl fv/1', 'autorouting', 'on');
add_line(sys, 'fl/1',         'A fl fv/2', 'autorouting', 'on');
add_line(sys, 'fv/1',         'A fl fv/3', 'autorouting', 'on');
add_line(sys, 'A fl fv/1',    'Fmax/1',    'autorouting', 'on');
add_line(sys, 'Fmax/1',       'F_CE/1',    'autorouting', 'on');
end

% -----------------------------------------------------------------------
function add_matlab_fcn(path, script, pos)
% Add a MATLAB Function block whose second argument "mp" is a parameter
% (read from the workspace, no input port), like the lecture model m01.
add_block('simulink/User-Defined Functions/MATLAB Function', path, 'Position', pos);
chart = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', path);
chart.Script = sprintf(script);
d = find(chart, '-isa', 'Stateflow.Data', 'Name', 'mp');
if isempty(d)
    d = Stateflow.Data(chart); d.Name = 'mp';
end
d.Scope = 'Parameter';
end
