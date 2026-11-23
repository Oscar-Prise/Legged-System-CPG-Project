%% run_h2_entrainment.m - Hypothesis H2: do two CPG walkers joined by the object fall into one rhythm?
%
% Two walkers with the SAME body and CPG parameters (pairtype 'identical'), leg-damper legs
% (the proposal's fallback: massless swing leg at a0, damper as the energy sink), CPG push-off
% with touchdown reset.  Two parts:
%
%   Part A  matched CPG clocks.  Sweep the initial conditions: which leg each walker stands on at
%           t = 0 (stance0 = [s1 s2], 1 = right, 2 = left) and the initial stride lag (lag0).
%           Expected (H2): the pair locks in step; with the same stance legs the lock reads as
%           pace (~0 deg), with opposite legs as trot (~180 deg).
%   Part B  mismatched CPG clocks.  Start in step and sweep omega2/omega1.  Expected (H2): the pair
%           still locks, with a lag that grows with the mismatch (possibly towards diagonal
%           sequence ~225 deg) until the lock is lost and the lag slips.
%           Run with the touchdown reset (the model as built) and, optionally, with a free-running
%           CPG + load feedback (reset 0, sigma 3), where the clock rate is not overwritten each step.
%
% Settings (set before running, or use the defaults):
%   h2_part     'A', 'B' or 'AB' (default)
%   h2_lags     Part A initial lags [deg], default 0:30:150
%   h2_stance   Part A stance combinations, default {[1 1], [1 2], [2 1], [2 2]}
%   h2_tEndA    Part A run time [s], default 60
%   h2_ratios   Part B omega2/omega1, default [0.9 0.95 0.97 0.98 0.99 1 1.01 1.02 1.03 1.05 1.08 1.1 1.15 1.2]
%   h2_cpg      Part B CPG variants, default {[1 0], [0 3]} = {[reset sigma]}: reset + no feedback,
%               free-running + feedback
%   h2_tEndB    Part B run time [s], default 60
% Output (folder ../results/h2/): h2_matched.mat, h2_mismatch.mat, fig_h2_matched.png, fig_h2_mismatch.png

if ~exist('h2_part', 'var') || isempty(h2_part), h2_part = 'AB'; end
if ~exist('h2_lags', 'var') || isempty(h2_lags), h2_lags = 0:30:150; end
if ~exist('h2_stance', 'var') || isempty(h2_stance), h2_stance = {[1 1], [1 2], [2 1], [2 2]}; end
if ~exist('h2_tEndA', 'var') || isempty(h2_tEndA), h2_tEndA = 60; end
if ~exist('h2_ratios', 'var') || isempty(h2_ratios), h2_ratios = [0.9 0.95 0.97 0.98 0.99 1 1.01 1.02 1.03 1.05 1.08 1.1 1.15 1.2]; end
if ~exist('h2_cpg', 'var') || isempty(h2_cpg), h2_cpg = {[1 0], [0 3]}; end
if ~exist('h2_tEndB', 'var') || isempty(h2_tEndB), h2_tEndB = 60; end
here = fileparts(mfilename('fullpath'));  res = fullfile(here, '..', 'results', 'h2');
if ~exist(res, 'dir'), mkdir(res); end
grey = [.5 .5 .5];

%% ---------------------------------------------------------------- Part A: matched clocks
if contains(h2_part, 'A')
    fprintf('\n===== H2 part A: matched CPG clocks, sweep stance legs x initial lag =====\n');
    A = struct('stance0', {}, 'lag0', {}, 'lag0_eff', {}, 'G', {});
    tic;
    for is = 1:numel(h2_stance)
        for il = 1:numel(h2_lags)
            preset = 'stage3_pair';  legmodel = 'damper';  pairtype = 'identical';
            lag0 = h2_lags(il);  stance0 = h2_stance{is};
            clear dl_set sigma_set reset_set phi0_set omega_ratio_set
            cpg_pair_params;
            R = sp_fixedstep_check(h2_tEndA, mp, ic);
            G = sp_gait_class(R, mp, h2_tEndA);
            a.stance0 = stance0;  a.lag0 = lag0;
            a.lag0_eff = mod(lag0 + 180*(stance0(2) ~= stance0(1)), 360);   % lag the labels give at t = 0
            a.G = G;  A(end+1) = a;   %#ok<SAGROW>
            fprintf('stance [%d %d] lag0 %3d (reads %3d) -> %-42s lag_end %6.1f  (%.0f s)\n', ...
                    stance0, lag0, a.lag0_eff, G.label, G.lag_end, toc);
        end
    end
    save(fullfile(res, 'h2_matched.mat'), 'A', 'h2_lags', 'h2_stance', 'h2_tEndA');

    % summary
    lab = arrayfun(@(a) a.G.label, A, 'UniformOutput', false);
    fprintf('\nPart A: %d starts, %d s each\n', numel(A), h2_tEndA);
    fprintf('  locked pace (in step, same legs):     %d\n', sum(strcmp(lab, 'pace')));
    fprintf('  locked trot (in step, opposite legs): %d\n', sum(strcmp(lab, 'trot')));
    fprintf('  still converging to in-step:          %d\n', sum(startsWith(lab, 'drifting to in-step')));
    fprintf('  other / half-step / fell:             %d\n', sum(~(strcmp(lab, 'pace') | strcmp(lab, 'trot') | startsWith(lab, 'drifting to in-step'))));

    plot_h2_matched(fullfile(res, 'h2_matched.mat'));
end

%% ---------------------------------------------------------------- Part B: mismatched clocks
if contains(h2_part, 'B')
    fprintf('\n===== H2 part B: CPG clock mismatch omega2/omega1, start in step =====\n');
    B = struct('ratio', {}, 'reset', {}, 'sigma', {}, 'G', {}, 'T1', {}, 'T2', {}, 'slip', {}, 'state', {});
    tic;
    for ic_ = 1:numel(h2_cpg)
        for ir = 1:numel(h2_ratios)
            preset = 'stage3_pair';  legmodel = 'damper';  pairtype = 'identical';
            lag0 = [];  stance0 = [1 1];
            clear dl_set phi0_set
            reset_set = h2_cpg{ic_}(1);  sigma_set = h2_cpg{ic_}(2);  omega_ratio_set = h2_ratios(ir);
            cpg_pair_params;
            R = sp_fixedstep_check(h2_tEndB, mp, ic);
            G = sp_gait_class(R, mp, h2_tEndB);
            b.ratio = h2_ratios(ir);  b.reset = reset_set;  b.sigma = sigma_set;  b.G = G;
            % stride times of each walker over the last 10 s (entrained = equal)
            t0 = max(R.t_end - 10, 0);
            for w = 1:2
                a = R.td(R.td(:,2) == w & R.td(:,3) == 1, 1);  a = a(a >= t0);
                if numel(a) >= 2, Tw = mean(diff(a)); else, Tw = NaN; end
                if w == 1, b.T1 = Tw; else, b.T2 = Tw; end
            end
            b.slip = G.drift/360;                                  % strides slipped per second
            if G.fell, b.state = sprintf('fell at %.1f s', G.t_end);
            elseif G.locked, b.state = 'LOCKED';
            else, b.state = sprintf('SLIPPING (%+.1f deg/s)', G.drift); end
            B(end+1) = b;   %#ok<SAGROW>
            fprintf('reset %d sigma %g  omega2/omega1 = %.3f -> %-24s lag_end %6.1f  T1 %.4f T2 %.4f  (%.0f s)\n', ...
                    reset_set, sigma_set, b.ratio, b.state, G.lag_end, b.T1, b.T2, toc);
        end
    end
    save(fullfile(res, 'h2_mismatch.mat'), 'B', 'h2_ratios', 'h2_cpg', 'h2_tEndB');

    plot_h2_mismatch(fullfile(res, 'h2_mismatch.mat'));
end
