%% make_result_videos.m - simulation videos (mp4 + gif) of the IC-sweep results, from ../results/ic_cases.mat
here = fileparts(mfilename('fullpath'));  res = fullfile(here, '..', 'results');
load(fullfile(res, 'ic_cases.mat'), 'C');
get = @(nm) C(strcmp({C.name}, nm));

% single-pair videos through the full animation (side view + footfall diagram): start and end of each run
c = get('walk_damper_lag60');
sp_anim_replay(c.R, c.mp, fullfile(res, 'video_walk_lag60_to_pace'), [0 6; 114 120], ...
    {'walking, start 60 deg apart: first 6 s', 'same run, t = 114-120 s: locked in step (pace)'});
c = get('walk_damper_lag60_tr');
sp_anim_replay(c.R, c.mp, fullfile(res, 'video_walk_lag60_to_trot'), [0 6; 114 120], ...
    {'same start, walker 2 on the other leg: first 6 s', 't = 114-120 s: same motion, labelled trot (180 deg)'});
c = get('walk_foot_lag90');
sp_anim_replay(c.R, c.mp, fullfile(res, 'video_walk_foot_lag90_falls'), [0 c.R.t(end)], ...
    {'foot-mass walkers, start half a step apart: the pair falls'});
c = get('run_lag30');
sp_anim_replay(c.R, c.mp, fullfile(res, 'video_run_lag30_to_halfstep'), [0 5; 15 20], ...
    {'running pair, start 30 deg apart: first 5 s', 't = 15-20 s: locked half a step apart (90 deg)'});
c = get('run_lag0');
sp_anim_replay(c.R, c.mp, fullfile(res, 'video_run_lag0_in_step'), [15 20], ...
    {'running pair started in step: stays in step (unstable balance)'});

% sweep montages: six starts side by side
Cw = C(startsWith({C.name}, 'sweep_damper_'));
sp_sweep_movie(Cw, fullfile(res, 'video_sweep_walking'), [0 6; 114 120], ...
    {'first 6 s', 't = 114-120 s'}, 'Walking pair (leg damper), identical walkers: six initial lags');
Cr = C(startsWith({C.name}, 'sweep_run_'));
sp_sweep_movie(Cr, fullfile(res, 'video_sweep_running'), [0 5; 15 20], ...
    {'first 5 s', 't = 15-20 s'}, 'Running pair, identical runners: six initial lags');
