%% make_h2_videos.m - videos (mp4 + gif) for the H2 runs: re-runs the cases (60 s), saves them thinned to
% ../results/h2/h2_cases.mat, then renders
%   video_h2_sweep_pace      Part A montage: same stance legs [1 1], lags 0..150 -> all lock as pace
%   video_h2_sweep_trot      Part A montage: opposite legs [1 2], same lags       -> all lock as trot
%   video_h2_pace_lag60      full animation, [1 1] lag 60: start and locked end (pace)
%   video_h2_trot_lag60      full animation, [1 2] lag 60: start and locked end (trot)
%   video_h2_mismatch_reset  Part B montage, model as built (reset, sigma 0): omega2/omega1 0.9..1.2
%   video_h2_mismatch_free   Part B montage, free-running CPG (reset 0, sigma 3): omega2/omega1 0.98..1.2
here = fileparts(mfilename('fullpath'));  res = fullfile(here, '..', 'results', 'h2');
if ~exist(res, 'dir'), mkdir(res); end
tEnd = 60;  dt_keep = 0.02;
lags = [0 30 60 90 120 150];
def = {};   % name, stance0, lag0, reset, sigma, ratio, panel title
for L = lags, def(end+1, :) = {sprintf('A_pace_%d', L), [1 1], L, 1, 0, 1, sprintf('same legs, start lag %d deg', L)}; end   %#ok<SAGROW>
for L = lags, def(end+1, :) = {sprintf('A_trot_%d', L), [1 2], L, 1, 0, 1, sprintf('opposite legs, start lag %d deg', L + 180)}; end   %#ok<SAGROW>
for r = [0.9 0.95 1.0 1.05 1.1 1.2], def(end+1, :) = {sprintf('B_reset_%g', r), [1 1], [], 1, 0, r, sprintf('reset on, \\omega_2/\\omega_1 = %.2f', r)}; end   %#ok<SAGROW>
for r = [0.98 1.0 1.02 1.05 1.1 1.2], def(end+1, :) = {sprintf('B_free_%g', r), [1 1], [], 0, 3, r, sprintf('free CPG, \\omega_2/\\omega_1 = %.2f', r)}; end   %#ok<SAGROW>

C = struct('name', {}, 'stance0', {}, 'lag0', {}, 'reset', {}, 'sigma', {}, 'ratio', {}, 'title', {}, 'R', {}, 'mp', {}, 'G', {});
tic;
for n = 1:size(def, 1)
    preset = 'stage3_pair';  legmodel = 'damper';  pairtype = 'identical';
    stance0 = def{n, 2};  lag0 = def{n, 3};  reset_set = def{n, 4};  sigma_set = def{n, 5};  omega_ratio_set = def{n, 6};
    clear dl_set phi0_set
    cpg_pair_params;
    R = sp_fixedstep_check(tEnd, mp, ic);
    G = sp_gait_class(R, mp, tEnd);
    c = struct('name', def{n, 1}, 'stance0', stance0, 'lag0', lag0, 'reset', reset_set, 'sigma', sigma_set, ...
               'ratio', omega_ratio_set, 'title', def{n, 7}, 'R', sp_thin(R, dt_keep), 'mp', mp, 'G', G);
    if isempty(c.lag0), c.lag0 = 0; end
    C(end+1) = c;   %#ok<SAGROW>
    fprintf('[%2d] %-14s -> %-42s lag_end %6.1f  (%.0f s)\n', n, c.name, G.label, G.lag_end, toc);
end
save(fullfile(res, 'h2_cases.mat'), 'C', '-v7.3');

get = @(nm) C(strcmp({C.name}, nm));
w = [0 6; tEnd - 6, tEnd];  cap = {'first 6 s', sprintf('t = %g-%g s', tEnd - 6, tEnd)};
sp_sweep_movie(C(startsWith({C.name}, 'A_pace_')), fullfile(res, 'video_h2_sweep_pace'), w, cap, ...
    'H2 A: matched clocks, same stance legs: six initial lags lock in step (pace, 0 deg)');
sp_sweep_movie(C(startsWith({C.name}, 'A_trot_')), fullfile(res, 'video_h2_sweep_trot'), w, cap, ...
    'H2 A: matched clocks, opposite stance legs: six initial lags lock in step (trot, 180 deg)');
c = get('A_pace_60');
sp_anim_replay(c.R, c.mp, fullfile(res, 'video_h2_pace_lag60'), w, ...
    {'H2 A: same stance legs, start 60 deg apart', 'locked in step: pace (0 deg)'});
c = get('A_trot_60');
sp_anim_replay(c.R, c.mp, fullfile(res, 'video_h2_trot_lag60'), w, ...
    {'H2 A: walker 2 on the other leg, start 240 deg', 'locked in step: trot (180 deg), same body motion'});
sp_sweep_movie(C(startsWith({C.name}, 'B_reset_')), fullfile(res, 'video_h2_mismatch_reset'), w, cap, ...
    'H2 B: clock mismatch with the touchdown reset: all stay in step');
sp_sweep_movie(C(startsWith({C.name}, 'B_free_')), fullfile(res, 'video_h2_mismatch_free'), w, cap, ...
    'H2 B: clock mismatch, free-running CPG (\sigma = 3): fall, slip, or re-lock one step apart');
fprintf('H2 videos written to %s\n', res);
