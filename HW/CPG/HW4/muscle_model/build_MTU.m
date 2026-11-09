function build_MTU(sys)
% BUILD_MTU  Add a muscle-tendon unit (CE in series with SE) subsystem.
%
%   build_MTU('model/MTU') creates a subsystem at that path with
%     inputs : 1) S  [ ]   neural stimulation
%              2) lm [m]   total muscle length = lCE + lSE
%     outputs: 1) Fm  [N]  muscle force (= SE force = CE force)
%              2) lCE [m]  contractile element length
%              3) lSE [m]  series element length
%              4) A   [ ]  activation
%
%   Structure (same as the lecture model m01): the SE sets the force from
%   its stretch, lSE = lm - lCE; the CE must carry the same force, so its
%   velocity follows from the inverted force-velocity relation
%       fv = Fm / (Fmax fl(lCE) A)  ->  vCE = vmax * fv^-1(fv),
%   and lCE is obtained by integrating vCE.
%
%   Needs the lecture functions CE_force_length_relation.m,
%   CE_inverse_force_velocity_relation.m and SE_force_length_relation.m.
%   Reads "mp" from the base workspace: Fmax, lopt, vmax, w, c, N, K,
%   lslack, eref, tauR, tauF, A0, lce0.

add_block('simulink/Ports & Subsystems/Subsystem', sys);
delete_line(sys, 'In1/1', 'Out1/1');
delete_block([sys '/In1']); delete_block([sys '/Out1']);

% ---- ports ---------------------------------------------------------
add_block('simulink/Sources/In1', [sys '/S'],   'Port', '1', 'Position', [30  60  60  74]);
add_block('simulink/Sources/In1', [sys '/lm'],  'Port', '2', 'Position', [30 300  60 314]);
add_block('simulink/Sinks/Out1',  [sys '/Fm'],  'Port', '1', 'Position', [820 300 850 314]);
add_block('simulink/Sinks/Out1',  [sys '/lCE'], 'Port', '2', 'Position', [820 420 850 434]);
add_block('simulink/Sinks/Out1',  [sys '/lSE'], 'Port', '3', 'Position', [820 360 850 374]);
add_block('simulink/Sinks/Out1',  [sys '/A'],   'Port', '4', 'Position', [820  60 850  74]);

% ---- excitation-contraction coupling (Problem 1.2) -----------------
add_block('simulink/Math Operations/Sum', [sys '/S - A'], 'Inputs', '+-', 'Position', [120 55 150 85]);
add_block('simulink/Sources/Constant', [sys '/1 over tauR'], 'Value', '1/mp.tauR', 'Position', [120 0 170 20]);
add_block('simulink/Sources/Constant', [sys '/1 over tauF'], 'Value', '1/mp.tauF', 'Position', [120 120 170 140]);
add_block('simulink/Signal Routing/Switch', [sys '/rising or falling'], ...
    'Criteria', 'u2 > Threshold', 'Threshold', '0', 'Position', [220 30 260 110]);
add_block('simulink/Math Operations/Product', [sys '/dA_dt'], 'Inputs', '2', 'Position', [310 50 340 110]);
add_block('simulink/Continuous/Integrator', [sys '/Integrator A'], 'InitialCondition', 'mp.A0', 'Position', [390 65 420 95]);
add_line(sys, 'S/1', 'S - A/1', 'autorouting', 'on');
add_line(sys, 'Integrator A/1', 'S - A/2', 'autorouting', 'on');
add_line(sys, 'S - A/1', 'rising or falling/2', 'autorouting', 'on');
add_line(sys, 'S - A/1', 'dA_dt/1', 'autorouting', 'on');
add_line(sys, '1 over tauR/1', 'rising or falling/1', 'autorouting', 'on');
add_line(sys, '1 over tauF/1', 'rising or falling/3', 'autorouting', 'on');
add_line(sys, 'rising or falling/1', 'dA_dt/2', 'autorouting', 'on');
add_line(sys, 'dA_dt/1', 'Integrator A/1', 'autorouting', 'on');
add_line(sys, 'Integrator A/1', 'A/1', 'autorouting', 'on');

% ---- series element: Fm = Fmax * fse(lSE / lslack) ------------------
add_block('simulink/Math Operations/Sum', [sys '/lm - lCE'], 'Inputs', '+-', 'Position', [120 295 150 325]);
add_block('simulink/Math Operations/Gain', [sys '/1 over lslack'], 'Gain', '1/mp.lslack', 'Position', [200 295 240 325]);
add_matlab_fcn([sys '/fse'], 'function fse = fcn(lse_nrm, mp)\nfse = SE_force_length_relation(lse_nrm, mp);', [300 288 380 332]);
add_block('simulink/Math Operations/Gain', [sys '/Fmax'], 'Gain', 'mp.Fmax', 'Position', [430 295 470 325]);
add_line(sys, 'lm/1', 'lm - lCE/1', 'autorouting', 'on');
add_line(sys, 'lm - lCE/1', '1 over lslack/1', 'autorouting', 'on');
add_line(sys, 'lm - lCE/1', 'lSE/1', 'autorouting', 'on');
add_line(sys, '1 over lslack/1', 'fse/1', 'autorouting', 'on');
add_line(sys, 'fse/1', 'Fmax/1', 'autorouting', 'on');
add_line(sys, 'Fmax/1', 'Fm/1', 'autorouting', 'on');

% ---- CE: force-length, then invert force-velocity to get vCE ---------
add_block('simulink/Math Operations/Gain', [sys '/1 over lopt'], 'Gain', '1/mp.lopt', 'Position', [120 420 160 450]);
add_matlab_fcn([sys '/fl'], 'function fl = fcn(lce_nrm, mp)\nfl = CE_force_length_relation(lce_nrm, mp);', [200 413 280 457]);
add_block('simulink/Math Operations/Product', [sys '/A fl'], 'Inputs', '2', 'Position', [330 400 360 460]);
add_block('simulink/Math Operations/Gain', [sys '/Fmax CE'], 'Gain', 'mp.Fmax', 'Position', [400 415 440 445]);
add_block('simulink/Sources/Constant', [sys '/eps'], 'Value', 'eps', 'Position', [400 480 440 500]);
add_block('simulink/Math Operations/Sum', [sys '/no div by 0'], 'Inputs', '++', 'Position', [480 415 510 445]);
add_block('simulink/Math Operations/Product', [sys '/fv = Fm over (Fmax fl A)'], 'Inputs', '*/', 'Position', [560 390 590 450]);
add_matlab_fcn([sys '/inverse fv'], 'function vce_nrm = fcn(fv, mp)\nvce_nrm = CE_inverse_force_velocity_relation(fv, mp);', [630 398 710 442]);
add_block('simulink/Math Operations/Gain', [sys '/vmax'], 'Gain', 'mp.vmax', 'Position', [630 480 670 510]);
add_block('simulink/Continuous/Integrator', [sys '/Integrator lCE'], 'InitialCondition', 'mp.lce0', 'Position', [720 480 750 510]);
add_line(sys, 'Integrator lCE/1', '1 over lopt/1', 'autorouting', 'on');
add_line(sys, '1 over lopt/1', 'fl/1', 'autorouting', 'on');
add_line(sys, 'Integrator A/1', 'A fl/1', 'autorouting', 'on');
add_line(sys, 'fl/1', 'A fl/2', 'autorouting', 'on');
add_line(sys, 'A fl/1', 'Fmax CE/1', 'autorouting', 'on');
add_line(sys, 'Fmax CE/1', 'no div by 0/1', 'autorouting', 'on');
add_line(sys, 'eps/1', 'no div by 0/2', 'autorouting', 'on');
add_line(sys, 'Fmax/1', 'fv = Fm over (Fmax fl A)/1', 'autorouting', 'on');
add_line(sys, 'no div by 0/1', 'fv = Fm over (Fmax fl A)/2', 'autorouting', 'on');
add_line(sys, 'fv = Fm over (Fmax fl A)/1', 'inverse fv/1', 'autorouting', 'on');
add_line(sys, 'inverse fv/1', 'vmax/1', 'autorouting', 'on');
add_line(sys, 'vmax/1', 'Integrator lCE/1', 'autorouting', 'on');
add_line(sys, 'Integrator lCE/1', 'lm - lCE/2', 'autorouting', 'on');
add_line(sys, 'Integrator lCE/1', 'lCE/1', 'autorouting', 'on');
end

% -----------------------------------------------------------------------
function add_matlab_fcn(path, script, pos)
% MATLAB Function block whose second argument "mp" is a parameter (as in m01).
add_block('simulink/User-Defined Functions/MATLAB Function', path, 'Position', pos);
chart = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', path);
chart.Script = sprintf(script);
d = find(chart, '-isa', 'Stateflow.Data', 'Name', 'mp');
if isempty(d)
    d = Stateflow.Data(chart); d.Name = 'mp';
end
d.Scope = 'Parameter';
end
