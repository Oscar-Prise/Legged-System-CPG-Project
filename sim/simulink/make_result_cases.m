%% make_result_cases.m - re-run representative starts of the IC sweep and save them (thinned) for
% the result figures and videos:  ../results/ic_cases.mat  (struct array C with R, mp, G, name, ...)
%
% Cases (two identical walkers unless noted):
%   walk_damper_lag60     walking, leg damper, start 60 deg, stance [1 1], 120 s  -> pace
%   walk_damper_lag60_tr  same start but walker 2 on the other leg               -> trot (same motion)
%   walk_damper_lag90     walking, leg damper, start 90 deg (unstable balance), 120 s
%   walk_foot_lag90       walking, foot mass, start 90 deg -> falls at 12.5 s
%   run_lag0              running pair, in step, 20 s  -> stays in step
%   run_lag30             running pair, 30 deg start, 20 s -> half a step apart
%   sweep_damper_*        walking damper, 120 s, lags 0 30 60 90 120 150 (montage video)
%   sweep_run_*           running, 20 s, lags 0 30 60 90 120 150 (montage video)

here = fileparts(mfilename('fullpath'));
outfile = fullfile(here, '..', 'results', 'ic_cases.mat');
dt_keep = 0.02;
C = struct('name', {}, 'preset', {}, 'legmodel', {}, 'lag0', {}, 'stance0', {}, 'tEnd', {}, 'R', {}, 'mp', {}, 'G', {});

def = {  % name                  preset         legmodel    lag0  stance  tEnd  dl
    'walk_damper_lag60',    'stage3_pair', 'damper',    60, [1 1], 120, [];
    'walk_damper_lag60_tr', 'stage3_pair', 'damper',    60, [1 2], 120, [];
    'walk_damper_lag90',    'stage3_pair', 'damper',    90, [1 1], 120, [];
    'walk_foot_lag90',      'stage3_pair', 'footmass',  90, [1 1],  30, [];
    'run_lag0',             'stage3_run',  'damper',     0, [1 1],  20, 0.04;
    'run_lag30',            'stage3_run',  'damper',    30, [1 1],  20, 0.04};
for L = [0 30 60 90 120 150]
    def(end+1, :) = {sprintf('sweep_damper_%d', L), 'stage3_pair', 'damper', L, [1 1], 120, []};   %#ok<SAGROW>
end
for L = [0 30 60 90 120 150]
    def(end+1, :) = {sprintf('sweep_run_%d', L), 'stage3_run', 'damper', L, [1 1], 20, 0.04};      %#ok<SAGROW>
end

tic;
for n = 1:size(def, 1)
    preset = def{n, 2};  legmodel = def{n, 3};  pairtype = 'identical';
    lag0 = def{n, 4};  stance0 = def{n, 5};  tEnd = def{n, 6};
    if isempty(def{n, 7}), clear dl_set; else, dl_set = def{n, 7}; end
    cpg_pair_params;
    R = sp_fixedstep_check(tEnd, mp, ic);
    G = sp_gait_class(R, mp, tEnd);
    c.name = def{n, 1};  c.preset = preset;  c.legmodel = legmodel;  c.lag0 = lag0;  c.stance0 = stance0;
    c.tEnd = tEnd;  c.R = sp_thin(R, dt_keep);  c.mp = mp;  c.G = G;
    C(end+1) = c;   %#ok<SAGROW>
    fprintf('[%2d] %-22s -> %-45s lag_end %6.1f  (%.0f s)\n', n, c.name, G.label, G.lag_end, toc);
end
save(outfile, 'C', '-v7.3');
fprintf('saved %s (%d cases)\n', outfile, numel(C));
