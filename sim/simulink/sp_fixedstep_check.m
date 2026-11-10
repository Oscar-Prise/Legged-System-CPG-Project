function R = sp_fixedstep_check(tEnd, mp, ic)
% SP_FIXEDSTEP_CHECK  Runs the two MATLAB Function block codes WITHOUT Simulink, the
% way the model executes them: fixed step dt, gait logic once per step (zero-order
% hold), RK4 (ode4) for the continuous states with the logic outputs held.
% Use it to check the block code, or to compare with the Simulink run.
%
%   preset = 'stage1_pair'; cpg_pair_params; R = sp_fixedstep_check(10);
%   R = sp_fixedstep_check(10, mp, ic);      % with explicit parameters
%
% R.Ebal = E_mech + arm loss + leg-damper loss + event losses - push-off work - hip work
% should stay constant (R.Ebal - R.Ebal(1) = numerical error).
% R.q_end, R.state_end: continuous and discrete state at the last step (for sp_start_state).

if nargin < 2, mp = evalin('base', 'mp'); end
if nargin < 3, ic = evalin('base', 'ic'); end
clear sp_logic_block                        % reset the persistent gait state
dt = mp.dt;  n = round(tEnd/dt);
q  = [ic.pos; ic.vel; 0; 0];                % [pos(5) vel(5) Earm Eleg]
dec = 10;  m = floor(n/dec) + 1;
R.t = zeros(m, 1);  R.pos = zeros(m, 5);  R.vel = zeros(m, 5);  R.phi = zeros(m, 2);
R.Eloss = zeros(m, 2);  R.legs = zeros(m, 36);  R.work = zeros(m, 4);  R.Ebal = zeros(m, 1);
R.td = zeros(0, 3);  R.stop = 0;
k = 0;
for s = 0:n
    t = s*dt;
    [legs, phi, stop, td, work, state] = sp_logic_block(q(1:5), q(6:10), mp);
    for i = 1:2, if td(i) > 0, R.td(end+1, :) = [t, i, td(i)]; end, end   %#ok<AGROW>
    if mod(s, dec) == 0
        k = k + 1;
        R.t(k) = t;  R.pos(k, :) = q(1:5).';  R.vel(k, :) = q(6:10).';
        R.phi(k, :) = phi.';  R.Eloss(k, :) = q(11:12).';  R.legs(k, :) = legs.';
        R.work(k, :) = work.';
        R.Ebal(k) = energy(q, legs, mp) + work(4) + q(11) + q(12) + work(3) - work(1) - work(2);
    end
    R.q_end = q;  R.state_end = state;  R.t_end = t;
    if stop > 0, R.stop = stop; break; end
    if s == n, break; end
    a = rhs(q, legs, mp);
    b = rhs(q + dt/2*a, legs, mp);
    c = rhs(q + dt/2*b, legs, mp);
    d = rhs(q + dt*c, legs, mp);
    q = q + dt/6*(a + 2*b + 2*c + d);
end
f = fieldnames(R);
for j = 1:numel(f)
    if any(strcmp(f{j}, {'td', 'stop', 'q_end', 'state_end', 't_end'})), continue; end
    R.(f{j}) = R.(f{j})(1:k, :);
end
end

function dq = rhs(q, legs, mp)
[acc, ~, ~, Ploss] = sp_eom_block(q(1:5), q(6:10), legs, mp);
dq = [q(6:10); acc; Ploss];
end

function E = energy(q, legs, mp)
% mechanical energy of bodies, object, arm springs and stance leg springs
p = q(1:5);  v = q(6:10);  E = 0;
for i = 1:mp.nW
    E = E + 0.5*mp.m(i)*(v(2*i-1)^2 + v(2*i)^2) + mp.m(i)*mp.g*p(2*i);
    for j = 1:2
        n = 2*(i-1) + j;
        if legs(n) > 0.5
            l = sqrt((p(2*i-1) - legs(4+n))^2 + p(2*i)^2);
            if legs(8+n) > l, E = E + 0.5*mp.k(i)*(legs(8+n) - l)^2; end
        end
    end
end
if mp.coupled > 0.5 && mp.nW > 1.5
    L = [p(5) - p(1); p(3) - p(5)];
    E = E + 0.5*mp.mt*v(5)^2 + 0.5*mp.ka*sum((L - mp.L0(:)).^2);
end
end
