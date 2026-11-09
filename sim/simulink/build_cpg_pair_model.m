%% build_cpg_pair_model.m - builds the Simulink model cpg_pair.slx
%
% Same idea as the HW4 scripts (p2_1_neuron.m ...): the model is created block by
% block with add_block / add_line, so it can be rebuilt any time.
% Block code lives in sp_logic_block.m and sp_eom_block.m and is copied into the
% two MATLAB Function blocks; the animation is the Level-2 S-function sp_anim.m.
%
%    Integrator pos <- Integrator vel <- [Leg and Arm Forces] <- legs
%          |                                   ^
%          +-- ZOH --> [Gait Logic and CPG] ---+--> Stop Simulation
%                                              +--> Animation (sp_anim)
%
% Run from the folder that holds this file.  Needs mp, ic, tmax, dtsim, rtf_anim
% in the base workspace (cpg_pair_params.m); runs it if they are missing.

here = fileparts(mfilename('fullpath'));
if ~exist('mp', 'var') || ~exist('ic', 'var'), cpg_pair_params; end

mdl = 'cpg_pair';
if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist(fullfile(here, [mdl '.slx']), 'file'), delete(fullfile(here, [mdl '.slx'])); end
new_system(mdl);  open_system(mdl);

%% Blocks -----------------------------------------------------------------------
% continuous states
add_block('simulink/Continuous/Integrator', [mdl '/Integrator vel'], ...
          'InitialCondition', 'ic.vel', 'Position', [560 140 600 180]);
add_block('simulink/Continuous/Integrator', [mdl '/Integrator pos'], ...
          'InitialCondition', 'ic.pos', 'Position', [660 140 700 180]);
add_block('simulink/Continuous/Integrator', [mdl '/Integrator losses'], ...
          'InitialCondition', '[0; 0]', 'Position', [560 300 600 340]);

% sampled copies of the states for the gait logic (runs once per step dtsim)
add_block('simulink/Discrete/Zero-Order Hold', [mdl '/ZOH pos'], ...
          'SampleTime', 'dtsim', 'Position', [140 40 180 70]);
add_block('simulink/Discrete/Zero-Order Hold', [mdl '/ZOH vel'], ...
          'SampleTime', 'dtsim', 'Position', [140 90 180 120]);

% MATLAB Function blocks
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/Gait Logic and CPG'], ...
          'Position', [240 30 400 150]);
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/Leg and Arm Forces'], ...
          'Position', [380 180 520 300]);
set_fcn_code([mdl '/Gait Logic and CPG'], fullfile(here, 'sp_logic_block.m'));
set_fcn_code([mdl '/Leg and Arm Forces'], fullfile(here, 'sp_eom_block.m'));

% stop on a fall
add_block('simulink/Sinks/Stop Simulation', [mdl '/Stop if fallen'], 'Position', [480 100 510 130]);

% animation (Level-2 MATLAB S-function, like m07_anim in HW2)
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', [mdl '/Animation'], ...
          'FunctionName', 'sp_anim', 'Parameters', 'mp, rtf_anim', 'Position', [880 20 980 80]);

% scopes
add_block('simulink/Sinks/Scope', [mdl '/Velocities'],  'Position', [880 150 910 180]);
add_block('simulink/Sinks/Scope', [mdl '/CPG phase'],   'Position', [560 60 590 90]);
add_block('simulink/Sinks/Scope', [mdl '/GRF'],         'Position', [620 370 650 400]);
add_block('simulink/Sinks/Scope', [mdl '/Arm forces'],  'Position', [620 430 650 460]);

% logging (decimated to 1 ms, touchdown flags at every step)
logs = {'pos_log',   'Integrator pos/1',     10, [880 220 940 240];
        'vel_log',   'Integrator vel/1',     10, [880 260 940 280];
        'phi_log',   'Gait Logic and CPG/2', 10, [880 300 940 320];
        'legs_log',  'Gait Logic and CPG/1', 10, [880 340 940 360];
        'td_log',    'Gait Logic and CPG/4',  1, [880 380 940 400];
        'grf_log',   'Leg and Arm Forces/2', 10, [880 420 940 440];
        'Farm_log',  'Leg and Arm Forces/3', 10, [880 460 940 480];
        'Eloss_log', 'Integrator losses/1',  10, [880 500 940 520]};
for k = 1:size(logs, 1)
    add_block('simulink/Sinks/To Workspace', [mdl '/' logs{k,1}], 'VariableName', logs{k,1}, ...
              'SaveFormat', 'Timeseries', 'Decimation', num2str(logs{k,3}), 'Position', logs{k,4});
end

%% Wiring -----------------------------------------------------------------------
L = {'Integrator pos/1',       'ZOH pos/1';
     'Integrator vel/1',       'ZOH vel/1';
     'ZOH pos/1',              'Gait Logic and CPG/1';
     'ZOH vel/1',              'Gait Logic and CPG/2';
     'Integrator pos/1',       'Leg and Arm Forces/1';
     'Integrator vel/1',       'Leg and Arm Forces/2';
     'Gait Logic and CPG/1',   'Leg and Arm Forces/3';
     'Leg and Arm Forces/1',   'Integrator vel/1';
     'Integrator vel/1',       'Integrator pos/1';
     'Leg and Arm Forces/4',   'Integrator losses/1';
     'Gait Logic and CPG/3',   'Stop if fallen/1';
     'Integrator pos/1',       'Animation/1';
     'Gait Logic and CPG/1',   'Animation/2';
     'Integrator vel/1',       'Velocities/1';
     'Gait Logic and CPG/2',   'CPG phase/1';
     'Leg and Arm Forces/2',   'GRF/1';
     'Leg and Arm Forces/3',   'Arm forces/1'};
for k = 1:size(L, 1)
    add_line(mdl, L{k,1}, L{k,2}, 'autorouting', 'on');
end
for k = 1:size(logs, 1)
    add_line(mdl, logs{k,2}, [logs{k,1} '/1'], 'autorouting', 'on');
end

%% Solver and callbacks (fixed step like the course models) -----------------------
set_param(mdl, 'StopTime', 'tmax', 'SolverType', 'Fixed-step', 'Solver', 'ode4', ...
          'FixedStep', 'dtsim', 'ReturnWorkspaceOutputs', 'on');
try, set_param(mdl, 'EnableMultiTasking', 'off'); catch, end   % single-tasking
set_param(mdl, 'PreLoadFcn', 'if ~exist(''mp'', ''var''), cpg_pair_params; end');
set_param(mdl, 'InitFcn',    'if ~exist(''mp'', ''var''), cpg_pair_params; end');

try
    note = Simulink.Annotation([mdl '/note']);
    note.Text = sprintf(['To switch stage:  preset = ''stage2_single'';  cpg_pair_params   then Run.\n' ...
                         'Presets: stage1_pair (passive Lanini pair), stage2_single (one CPG walker),\n' ...
                         'stage3_pair (two CPG walkers).  After a run:  plot_cpg_pair_results']);
    note.Position = [40 380 560 440];
catch
end

save_system(mdl, fullfile(here, [mdl '.slx']));
fprintf('built %s.slx\n', mdl);


%% ---- local function ----------------------------------------------------------
function set_fcn_code(blk, file)
% copy the code of FILE into the MATLAB Function block BLK and make 'mp' a parameter
ch = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', blk);
ch.Script = fileread(file);
d = find(ch, '-isa', 'Stateflow.Data', 'Name', 'mp');
if isempty(d)                      % not created yet by this MATLAB release: add it
    d = Stateflow.Data(ch);
    d.Name = 'mp';
end
d.Scope = 'Parameter';
end
