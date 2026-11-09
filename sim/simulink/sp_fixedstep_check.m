function R = sp_fixedstep_check(tEnd)
% SP_FIXEDSTEP_CHECK  Runs the two MATLAB Function block codes WITHOUT Simulink, the
% way the model executes them: fixed step dt, gait logic once per step (zero-order
% hold), RK4 (ode4) for the continuous states with the logic outputs held.
% Use it to check the block code, or to compare with the Simulink run.
%
%   preset = 'stage1_pair'; cpg_pair_params; R = sp_fixedstep_check(10);

mp = evalin('base', 'mp');  ic = evalin('base', 'ic');
clear sp_logic_block                        % reset the persistent gait state
dt = mp.dt;  n = round(tEnd/dt);
q  = [ic.pos; ic.vel; 0; 0];                % [pos(5) vel(5) Earm Eleg]
dec = 10;  m = floor(n/dec) + 1;
R.t = zeros(m, 1);  R.pos = zeros(m, 5);  R.vel = zeros(m, 5);  R.phi = zeros(m, 2);
R.Eloss = zeros(m, 2);  R.legs = zeros(m, 12);  R.td = zeros(0, 3);  R.stop = 0;
k = 0;
for s = 0:n
    t = s*dt;
    [legs, phi, stop, td] = sp_logic_block(q(1:5), q(6:10), mp);
    for i = 1:2, if td(i) > 0, R.td(end+1, :) = [t, i, td(i)]; end, end   %#ok<AGROW>
    if mod(s, dec) == 0
        k = k + 1;
        R.t(k) = t;  R.pos(k, :) = q(1:5).';  R.vel(k, :) = q(6:10).';
        R.phi(k, :) = phi.';  R.Eloss(k, :) = q(11:12).';  R.legs(k, :) = legs.';
    end
    if stop > 0, R.stop = stop; break; end
    a = rhs(q, legs, mp);
    b = rhs(q + dt/2*a, legs, mp);
    c = rhs(q + dt/2*b, legs, mp);
    d = rhs(q + dt*c, legs, mp);
    q = q + dt/6*(a + 2*b + 2*c + d);
end
f = fieldnames(R);
for j = 1:numel(f)
    if any(strcmp(f{j}, {'td', 'stop'})), continue; end
    R.(f{j}) = R.(f{j})(1:k, :);
end
end

function dq = rhs(q, legs, mp)
[acc, ~, ~, Ploss] = sp_eom_block(q(1:5), q(6:10), legs, mp);
dq = [q(6:10); acc; Ploss];
end
