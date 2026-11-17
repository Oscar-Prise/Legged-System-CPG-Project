%% sweep_cpg_phase.m - two IDENTICAL walkers started mechanically in step; sweep the initial CPG
% phase of each walker (phi1_0, phi2_0).  Both walkers start at the same apex state (leg 1 in
% stance), the only difference is where each CPG clock starts.
%   sw_legmodel ('damper'), sw_sigma (load-feedback gain), sw_reset (1 = reset at touchdown,
%   0 = free-running CPG), sw_phis (grid, default 0:pi/4:7pi/4), sw_tEnd (30 s), sw_tag
% Results: ../results/cpg_phase_sweep_<tag>.mat; plot with plot_cpg_phase_sweep.m
if ~exist('sw_legmodel', 'var') || isempty(sw_legmodel), sw_legmodel = 'damper'; end
if ~exist('sw_sigma', 'var'),  sw_sigma = 0; end
if ~exist('sw_reset', 'var'),  sw_reset = 1; end
if ~exist('sw_phis', 'var') || isempty(sw_phis), sw_phis = 0:pi/4:7*pi/4; end
if ~exist('sw_tEnd', 'var') || isempty(sw_tEnd), sw_tEnd = 30; end
if ~exist('sw_tag', 'var'),  sw_tag = sprintf('%s_sig%g_reset%d', sw_legmodel, sw_sigma, sw_reset); end
here = fileparts(mfilename('fullpath'));
out = fullfile(here, '..', 'results', ['cpg_phase_sweep_' sw_tag]);

n = numel(sw_phis);
lag_end = nan(n);  offset_end = nan(n);  maxoff = nan(n);  fell = false(n);  tend = nan(n);
drift = nan(n);  v1 = nan(n);  tsettle = nan(n);  label = cell(n);
tic;
for a = 1:n
    for b = 1:n
        preset = 'stage3_pair';  legmodel = sw_legmodel;  pairtype = 'identical';  lag0 = [];  stance0 = [1 1];
        sigma_set = sw_sigma;  reset_set = sw_reset;  phi0_set = [sw_phis(a) sw_phis(b)];
        clear dl_set
        cpg_pair_params;
        R = sp_fixedstep_check(sw_tEnd, mp, ic);
        G = sp_gait_class(R, mp, sw_tEnd);
        lag_end(a, b) = G.lag_end;  offset_end(a, b) = G.offset;  drift(a, b) = G.drift;
        fell(a, b) = G.fell;  tend(a, b) = G.t_end;  v1(a, b) = G.v1;  label{a, b} = G.label;
        if ~isempty(G.lag)
            p = mod(G.lag, 180);  off = min(p, 180 - p);          % physical offset over time
            maxoff(a, b) = max(off);
            k = find(off > 5, 1, 'last');                          % last stride more than 5 deg off in-step
            if isempty(k), tsettle(a, b) = 0; elseif k < numel(off), tsettle(a, b) = G.tl(k + 1); end
        end
        fprintf('phi0 = [%.2f %.2f] -> %-45s max offset %5.1f  settle %5.1f s  (%.0f s)\n', ...
                sw_phis(a), sw_phis(b), G.label, maxoff(a, b), tsettle(a, b), toc);
    end
end
sweep = struct('phis', sw_phis, 'legmodel', sw_legmodel, 'sigma', sw_sigma, 'reset', sw_reset, 'tEnd', sw_tEnd, ...
               'lag_end', lag_end, 'offset_end', offset_end, 'maxoff', maxoff, 'fell', fell, 'tend', tend, ...
               'drift', drift, 'v1', v1, 'tsettle', tsettle, 'label', {label});
save([out '.mat'], 'sweep');
fprintf('saved %s.mat\n', out);
