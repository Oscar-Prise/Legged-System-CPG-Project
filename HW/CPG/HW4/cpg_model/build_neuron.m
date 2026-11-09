function build_neuron(sys, x_init)
% BUILD_NEURON  Add a Matsuoka neuron subsystem to a Simulink model.
%
%   build_neuron('model/e_L')         membrane potential starts at 0
%   build_neuron('model/e_L', '1')    membrane potential starts at 1
%
%   Model (Matsuoka 1985/1987, one neuron with self-adaptation):
%       y   = max(x, 0)                              firing rate (output)
%       x'  = (1/tau_x) * (x0 + sum_i w_i y_i - w' * xa - x)   membrane potential
%       xa' = (1/tau_a) * (y - xa)                   adaptation ("fatigue")
%
%   Ports:   inputs  1) x0        central (tonic) drive
%                    2) sum_wy    weighted input from other neurons
%            outputs 1) y         firing rate
%                    2) x         membrane potential
%                    3) xa        adaptation state
%
%   Each first-order ODE  tau * dz/dt = (inputs) - z  is built as
%       Sum  ->  Gain 1/tau  ->  Integrator  (with z fed back into the Sum),
%   the same pattern as the activation dynamics in Problem 1.2.
%
%   Reads the parameter struct "np" from the base workspace:
%       np.taux, np.taua, np.wp

if nargin < 2, x_init = '0'; end

add_block('simulink/Ports & Subsystems/Subsystem', sys);
delete_line(sys, 'In1/1', 'Out1/1');
delete_block([sys '/In1']); delete_block([sys '/Out1']);

% ---- ports ------------------------------------------------------------
add_block('simulink/Sources/In1', [sys '/x0'],     'Port', '1', 'Position', [30  80  60  94]);
add_block('simulink/Sources/In1', [sys '/sum_wy'], 'Port', '2', 'Position', [30 120  60 134]);
add_block('simulink/Sinks/Out1',  [sys '/y'],      'Port', '1', 'Position', [620 100 650 114]);
add_block('simulink/Sinks/Out1',  [sys '/x'],      'Port', '2', 'Position', [620 160 650 174]);
add_block('simulink/Sinks/Out1',  [sys '/xa'],     'Port', '3', 'Position', [620 260 650 274]);

% ---- membrane potential:  tau_x * dx/dt = x0 + sum_wy - w'*xa - x ------
add_block('simulink/Math Operations/Sum', [sys '/sum x'], 'Inputs', '++--', ...
    'IconShape', 'rectangular', 'Position', [140 80 170 150]);
add_block('simulink/Math Operations/Gain', [sys '/1 over taux'], 'Gain', '1/np.taux', 'Position', [220 100 260 130]);
add_block('simulink/Continuous/Integrator', [sys '/int x'], 'InitialCondition', x_init, 'Position', [310 100 340 130]);

% ---- output nonlinearity:  y = max(x, 0) --------------------------------
add_block('simulink/Discontinuities/Saturation', [sys '/max(x,0)'], ...
    'UpperLimit', 'inf', 'LowerLimit', '0', 'Position', [400 100 430 130]);

% ---- adaptation:  tau_a * dxa/dt = y - xa --------------------------------
add_block('simulink/Math Operations/Sum', [sys '/sum xa'], 'Inputs', '+-', 'Position', [400 250 430 280]);
add_block('simulink/Math Operations/Gain', [sys '/1 over taua'], 'Gain', '1/np.taua', 'Position', [460 250 500 280]);
add_block('simulink/Continuous/Integrator', [sys '/int xa'], 'InitialCondition', '0', 'Position', [540 250 570 280]);
add_block('simulink/Math Operations/Gain', [sys '/w prime'], 'Gain', 'np.wp', ...
    'Orientation', 'left', 'Position', [220 200 260 230]);

% ---- wiring --------------------------------------------------------------
add_line(sys, 'x0/1',          'sum x/1',       'autorouting', 'on');
add_line(sys, 'sum_wy/1',      'sum x/2',       'autorouting', 'on');
add_line(sys, 'w prime/1',     'sum x/3',       'autorouting', 'on');   % - w' xa
add_line(sys, 'int x/1',       'sum x/4',       'autorouting', 'on');   % - x   (leak)
add_line(sys, 'sum x/1',       '1 over taux/1', 'autorouting', 'on');
add_line(sys, '1 over taux/1', 'int x/1',       'autorouting', 'on');
add_line(sys, 'int x/1',       'max(x,0)/1',    'autorouting', 'on');
add_line(sys, 'int x/1',       'x/1',           'autorouting', 'on');
add_line(sys, 'max(x,0)/1',    'y/1',           'autorouting', 'on');
add_line(sys, 'max(x,0)/1',    'sum xa/1',      'autorouting', 'on');   % + y
add_line(sys, 'int xa/1',      'sum xa/2',      'autorouting', 'on');   % - xa (leak)
add_line(sys, 'sum xa/1',      '1 over taua/1', 'autorouting', 'on');
add_line(sys, '1 over taua/1', 'int xa/1',      'autorouting', 'on');
add_line(sys, 'int xa/1',      'xa/1',          'autorouting', 'on');
add_line(sys, 'int xa/1',      'w prime/1',     'autorouting', 'on');
end
