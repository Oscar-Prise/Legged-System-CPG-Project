function P = slip_params(set_name)
% SLIP_PARAMS  Parameters for the (paired) bipedal spring-mass walker.
%
%   P = slip_params('proposal')  walker of Geyer 2006 Fig. 3A + arms of Lanini 2017 Fig. 7
%   P = slip_params('lanini')    parameter set of Lanini 2017 Table 6
%
% Sources (all in /Paper):
%   Geyer, Seyfarth & Blickhan 2006, Fig. 3A: alpha0 = 69 deg, k = 14 kN/m, Es = 816 J
%   Lanini et al. 2017, Fig. 7 caption (Step 3): arms k = 3.5e3 N/m, b = 320 Ns/m
%   Lanini et al. 2017, Table 6: kleg = 17.9e3 N/m, alpha = 72.4 deg, karm = 3100 N/m, barm = 200 Ns/m
%   Lanini et al. 2017, Sec. 4.2: m1 = m2 = 80 kg, l1 = l2 = 1 m
%   Object mass: 8 kg in the experiments (Lanini Sec. 2), 7.5 kg in Table 4. We use 8 kg.

if nargin < 1, set_name = 'proposal'; end

P.g  = 9.81;      % [m/s^2]
P.m  = 80;        % [kg]   walker mass (both walkers)
P.l0 = 1.0;       % [m]    leg rest length
P.mt = 8;         % [kg]   carried object

switch lower(set_name)
    case 'proposal'
        P.k   = 14e3;          % [N/m]   leg stiffness
        P.a0  = 69*pi/180;     % [rad]   angle of attack (swing leg w.r.t. ground)
        P.ka  = 3.5e3;         % [N/m]   arm stiffness  (per arm)
        P.ba  = 320;           % [Ns/m]  arm damping    (per arm)
        P.E   = 816;           % [J]     nominal system energy per walker (Geyer 2006 Fig. 3)
    case 'lanini'
        P.k   = 17.9e3;
        P.a0  = 72.4*pi/180;
        P.ka  = 3100;
        P.ba  = 200;
        P.E   = 825;           % [J]     Lanini Table 5, Steps 1 and 3
    otherwise
        error('unknown parameter set %s', set_name);
end

% Stage 2 additions, all OFF here so Stage 1 is the passive model.
% run_stage2_cpg_walker.m switches them on.
P.c       = 0;          % [Ns/m]  leg damper (energy sink of a single walker)
P.cpg     = false;      %         CPG push-off on/off
P.dl      = 0;          % [m]     push-off: leg rest length extends by dl
P.omega   = 2*pi/1.0;   % [rad/s] CPG intrinsic frequency (one cycle = one stride); scalar or [w1 w2]
P.sigma   = 0;          % [rad/s per body weight]  load feedback gain (Owaki & Ishiguro 2017 Eq. 1)
P.beta    = 0.69;       %         nominal duty factor -> reset phase th_TD = 2 pi (1 - beta) (Aoi & Tsuchiya 2007)
P.th_TD   = 2*pi*(1 - P.beta);   % [rad] leg phase after touchdown reset
P.th_on   = P.th_TD + pi + 0.1;  % [rad] push-off starts (leg phase); th_TD + pi = contralateral touchdown.
                                  %       Keep omega * T_step < pi + 0.1 so the phase never enters the ramp
                                  %       before the reset (otherwise lambda jumps under load)
P.th_ramp = 0.6;                  % [rad] ramp duration
P.th_hold = pi - 0.3;             % [rad] u returns to 0 this long after th_on (leg in swing by then);
                                  %       must stay below mod(th_TD - th_on, 2 pi) = pi - 0.1 so legs land at l0

% simulation settings
P.z_fail = 0.5 * P.l0;   % [m] CoM below this = fall
end
