function [ic, mp] = sp_start_state(mp, ic, lag0, stance0)
% SP_START_STATE  Start the pair with a chosen stride lag and chosen stance legs.
%
%   [ic, mp] = sp_start_state(mp, ic, lag0)             walker 2 lags walker 1 by lag0 deg of a stride
%   [ic, mp] = sp_start_state(mp, ic, lag0, stance0)    and walker i starts on leg stance0(i) (1 or 2)
%
% Each walker first walks alone (same block code, no arms) from its apex start:
%   walker 1 for 2 of its strides, walker 2 for 2 strides + (360 - lag0)/360 of a stride.
% Their end states become the pair's start state (walker 1 at x = ic.pos(1), walker 2 at
% x = ic.pos(3), object half way, arms relaxed).  Walker 2 is then ahead in its cycle, so its
% leg-1 touchdown follows walker 1's by about lag0/360 of a stride.  The lag actually reached
% is shown in the animation and by plot_cpg_pair_results.
%
% stance0(i) = 2 swaps the leg labels of walker i (the leg that is in stance at t = 0 becomes
% leg 2 = the left leg): the leg columns of its state are exchanged and its CPG phase is
% shifted by pi (theta_j = phi + (j-1) pi, so the phase of each physical leg is unchanged).
% The body motion is the same; only the footfall labels and the reported stride lag change
% (by 180 deg).  With stance0 = [1 2] and lag0 = 0 the pair starts in a trot-like pattern.
%
% Note (Lanini 2017 Sec. 5.3): in 2-D, a 180 deg stride lag (trot) and 0 deg (pace) give the
% same body motion; only the leg labels differ.  The largest physical offset is 90 deg (half a step).

if nargin < 4 || isempty(stance0), stance0 = [1 1]; end
if isempty(lag0), lag0 = 0; end
nS = 2;                                   % strides walked alone before the pair starts
X  = [ic.pos(1), ic.pos(3)];
per = {'m', 'l0', 'k', 'a0', 'c', 'dl', 'mf', 'Ksw', 'Dsw', 'h_sw', 'omega'};
for i = 1:mp.nW
    ms = mp;                              % walker i alone, in the walker-1 slot
    for f = per, ms.(f{1}) = mp.(f{1})(i)*[1 1]; end
    ms.nW = 1;  ms.coupled = 0;  ms.init_use = [0 0];
    ms.foot0 = mp.foot0(i)*[1 1];  ms.phi0 = mp.phi0(i)*[1 1];
    if mp.init_use(i) > 0.5                % a start state was prepared already (flight start)
        ms.init = zeros(1, 50);  ms.init(1:25) = mp.init(25*(i-1) + (1:25));  ms.init_use = [1 0];
    end
    ics.pos = [ic.pos(2*i-1); ic.pos(2*i); 0; 0; ic.pos(2*i-1)];
    ics.vel = [ic.vel(2*i-1); ic.vel(2*i); 0; 0; ic.vel(2*i-1)];

    R = sp_fixedstep_check(3, ms, ics);   % stride time of this walker
    if R.stop > 0, error('sp_start_state: walker %d falls when walking alone', i); end
    t1 = R.td(R.td(:,3) == 1, 1);
    T  = mean(diff(t1(max(1, end-2):end)));
    tw = nS*T;
    if i == 2, tw = tw + mod(-lag0, 360)/360*T; end

    R = sp_fixedstep_check(tw, ms, ics);  % walk alone until the start state
    if R.stop > 0, error('sp_start_state: walker %d falls when walking alone', i); end
    S = R.state_end(1:25).';
    shift = X(i) - R.q_end(1);
    S(3:4)   = S(3:4) + shift*S(1:2);     % stance feet (swing entries stay 0)
    S(9:10)  = S(9:10) + shift;           % feet
    S(17:18) = S(17:18) + shift;          % take-off points of the swing feet
    S(21:24) = S(21:24) - R.t_end;        % touchdown / take-off times relative to the start
    if stance0(i) == 2                    % swap the leg labels of this walker
        for p = 1:2:23, S([p p+1]) = S([p+1 p]); end
        S(25) = mod(S(25) + pi, 2*pi);
    elseif stance0(i) ~= 1
        error('sp_start_state: stance0(%d) must be 1 or 2', i);
    end
    mp.init(25*(i-1) + (1:25)) = S;
    mp.init_use(i) = 1;
    ic.pos(2*i-1) = X(i);  ic.pos(2*i) = R.q_end(2);
    ic.vel(2*i-1) = R.q_end(6);  ic.vel(2*i) = R.q_end(7);
    fprintf('sp_start_state: walker %d stride %.3f s, walked alone %.3f s, starts on leg %d\n', ...
            i, T, tw, stance0(i));
end
ic.pos(5) = 0.5*(ic.pos(1) + ic.pos(3));
ic.vel(5) = 0.5*(ic.vel(1) + ic.vel(3));
mp.L0 = [ic.pos(5) - ic.pos(1), ic.pos(3) - ic.pos(5)];
mp.stance0 = stance0;
clear sp_logic_block                      % the pair run starts with a fresh gait state
end
