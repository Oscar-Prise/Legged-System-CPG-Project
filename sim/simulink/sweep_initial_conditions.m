%% sweep_initial_conditions.m - two IDENTICAL CPG walkers: sweep the initial conditions
%
% Both walkers have exactly the same parameters and the same CPG clock (pairtype 'identical',
% dE = 0).  What is swept is only how the pair starts:
%   stance0 = [s1 s2]  which leg each walker stands on at t = 0 (1 or 2; can differ)
%   lag0               stride lag of walker 2 behind walker 1 at t = 0 [deg]
% Each case runs the block code without Simulink (sp_fixedstep_check), then sp_gait_class
% measures the stride lag over the last 10 s, tests for a lock and names the gait
% (pace ~0, trot ~180, diagonal sequence ~225, half-step ~90/270 deg; or drifting / fell).
%
% Settings can be set before running the script (for batch runs), e.g.
%   sw_legmodel = 'damper'; sw_lags = 0:15:165; sw_stance = {[1 1], [1 2]}; sweep_initial_conditions
% Results go to ../results/ic_sweep_<legmodel>[_tag].mat (see plot_ic_sweep.m for table and figures).

if ~exist('sw_legmodel', 'var') || isempty(sw_legmodel), sw_legmodel = 'damper'; end
if ~exist('sw_tEnd', 'var')     || isempty(sw_tEnd),     sw_tEnd = 30; end
if ~exist('sw_lags', 'var')     || isempty(sw_lags),     sw_lags = 0:15:165; end
if ~exist('sw_stance', 'var')   || isempty(sw_stance),   sw_stance = {[1 1], [1 2], [2 1], [2 2]}; end
if ~exist('sw_tag', 'var'),      sw_tag = ''; end
if ~exist('sw_dE', 'var'),       sw_dE = 0; end          % energy difference [J] (0 = identical start energy)
if ~exist('sw_preset', 'var') || isempty(sw_preset), sw_preset = 'stage3_pair'; end   % or 'stage3_run'
if ~exist('sw_dl', 'var'),       sw_dl = []; end         % push-off dl override [m] ([] = preset value)
if ~exist('sw_sigma', 'var'),    sw_sigma = []; end      % CPG load-feedback gain sigma ([] = preset value, 0)
if ~exist('sw_out', 'var') || isempty(sw_out)
    sw_out = fullfile(fileparts(mfilename('fullpath')), '..', 'results', ['ic_sweep_' sw_legmodel sw_tag]);
    if ~exist('sw_preset', 'var') || isempty(sw_preset) || strcmp(sw_preset, 'stage3_pair'), else
        sw_out = fullfile(fileparts(mfilename('fullpath')), '..', 'results', ['ic_sweep_' sw_preset '_' sw_legmodel sw_tag]);
    end
end

S = struct('stance0', {}, 'lag0', {}, 'G', {}, 'mp', {});
tic;
for is = 1:numel(sw_stance)
    for il = 1:numel(sw_lags)
        preset = sw_preset;  legmodel = sw_legmodel;  pairtype = 'identical';
        lag0 = sw_lags(il);  stance0 = sw_stance{is};  dE_set = sw_dE;
        if ~isempty(sw_dl), dl_set = sw_dl; else, clear dl_set; end
        if ~isempty(sw_sigma), sigma_set = sw_sigma; else, clear sigma_set; end
        cpg_pair_params;
        R = sp_fixedstep_check(sw_tEnd, mp, ic);
        G = sp_gait_class(R, mp, sw_tEnd);
        n = numel(S) + 1;
        S(n).stance0 = stance0;  S(n).lag0 = lag0;  S(n).G = G;  S(n).mp = mp;
        fprintf('[%3d] stance [%d %d], lag0 %3d deg -> %-28s lag_end %6.1f  offset %5.1f  drift %+6.2f deg/s  (%.0f s elapsed)\n', ...
                n, stance0(1), stance0(2), lag0, G.label, G.lag_end, G.offset, G.drift, toc);
    end
end
sweep.S = S;  sweep.legmodel = sw_legmodel;  sweep.tEnd = sw_tEnd;  sweep.lags = sw_lags;
sweep.stance = sw_stance;  sweep.dE = sw_dE;  sweep.preset = sw_preset;  sweep.dl = sw_dl;  sweep.sigma = sw_sigma;
sweep.date = datestr(now);   %#ok<TNOW1,DATST>
[d, ~] = fileparts(sw_out);  if ~exist(d, 'dir'), mkdir(d); end
save([sw_out '.mat'], 'sweep');
fprintf('saved %s.mat (%d runs, %.0f s)\n', sw_out, numel(S), toc);
