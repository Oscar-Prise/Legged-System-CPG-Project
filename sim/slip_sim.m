function out = slip_sim(P, W, opt)
% SLIP_SIM  Hybrid simulation of one or two bipedal spring-mass walkers,
% optionally coupled through an object with spring-damper arms (Lanini 2017).
%
%   out = slip_sim(P, W, opt)
%
%   P     parameters (slip_params)
%   W     1x1 or 1x2 struct array of walker states (slip_walker_at_apex)
%         (P.cpg = true adds the CPG push-off and the touchdown phase reset, Stage 2)
%   opt   fields (all optional):
%         tEnd       [s]  final time                          (default 20)
%         coupled         true -> arms + object active        (default: numel(W)==2)
%         xo0, xod0       object position / speed at t = 0    (default: midway, mean speed)
%         nApexStop       stop after this many recorded apexes of walker 1 (default inf)
%         RelTol, AbsTol, MaxStep                              ode45 settings
%
%   out.seg     integration segments (t, y, and the walker contact/foot data valid on it)
%   out.ev      event log: t, walker, type ('TD','TO','APEX','FAIL...'), leg, state
%   out.W       final discrete walker states
%   out.status  'ok' or the failure reason
%   out.L0      arm rest lengths

nW = numel(W);
if nargin < 3, opt = struct(); end
opt = set_default(opt, 'tEnd', 20);
opt = set_default(opt, 'coupled', nW == 2);
opt = set_default(opt, 'nApexStop', inf);
opt = set_default(opt, 'RelTol', 1e-8);
opt = set_default(opt, 'AbsTol', 1e-9);
opt = set_default(opt, 'MaxStep', 1e-2);
if nW == 2
    opt = set_default(opt, 'xo0',  (W(1).x + W(2).x)/2);
    opt = set_default(opt, 'xod0', (W(1).xd + W(2).xd)/2);
    opt.L0 = [opt.xo0 - W(1).x, W(2).x - opt.xo0];   % arms start at rest (Lanini Sec. 3.2)
else
    opt = set_default(opt, 'xo0', 0);  opt = set_default(opt, 'xod0', 0);
    opt.L0 = [0 0];  opt.coupled = false;
end

I  = slip_idx(nW);
if P.cpg && slip_pushoff(P.th_TD, P) ~= 0
    error('push-off window covers the touchdown phase th_TD: legs would land extended (check th_on/th_hold)');
end
y0 = zeros(I.n, 1);
for i = 1:nW
    y0(4*i-3:4*i) = [W(i).x; W(i).z; W(i).xd; W(i).zd];
    y0(I.phi(i))  = W(i).phi;
end
y0([I.xo I.xod]) = [opt.xo0; opt.xod0];

t0 = 0;  seg = {};  ev = {};  status = 'ok';  nApex = 0;
names_ss = {'TD', 'APEX', 'FAIL flight in single support', 'FAIL walked backward'};
names_ds = {'TO', 'FAIL leading leg lifted off', 'FAIL walked backward', 'FAIL collapsed'};

while t0 < opt.tEnd - 1e-12
    ode_opt = odeset('RelTol', opt.RelTol, 'AbsTol', opt.AbsTol, 'MaxStep', opt.MaxStep, ...
                     'Events', @(t, y) slip_events(t, y, W, P, opt));
    [t, Y, te, ye, ie] = ode45(@(t, y) slip_rhs(t, y, W, P, opt), [t0 opt.tEnd], y0, ode_opt);
    if isempty(ie)                                                                 % reached tEnd
        seg{end+1} = struct('t', t, 'y', Y, 'W', W);                               %#ok<AGROW>
        break;
    end
    % cut the segment at the EARLIEST event (Octave's ode45 may run past a terminal
    % event that occurs in its first step; MATLAB stops there anyway)
    [tE, n1] = min(te);
    yE   = ye(n1, :).';
    hits = unique(ie(abs(te - tE) < 1e-9)).';
    keep = t < tE - 1e-12;
    t = [t(keep); tE];  Y = [Y(keep, :); yE.'];
    seg{end+1} = struct('t', t, 'y', Y, 'W', W);                                   %#ok<AGROW>
    % events of other walkers that cross at (almost) the same instant: the solver
    % reports only one, and after the restart the other sits at zero and is missed
    [v1, ~, d1] = slip_events(tE, yE, W, P, opt);
    v0 = slip_events(t(max(end-1, 1)), Y(max(end-1, 1), :).', W, P, opt);
    near = find(abs(v1) < 1e-6 & v0 .* d1 < 0).';
    hits = unique([hits, near]);
    stop = false;
    for idx = hits
        i = ceil(idx / 4);  k = idx - 4*(i-1);
        ss = sum(W(i).contact) == 1;
        if ss, name = names_ss{k}; else, name = names_ds{k}; end
        x = yE(4*i-3);
        E_before = slip_energy(yE, W, P, opt);
        switch name
            case 'TD'                                    % swing leg lands
                j = find(~W(i).contact);
                W(i).foot(j) = x + P.l0*cos(P.a0);
                W(i).contact(j) = true;
                leg = j;
                if P.cpg                                 % phase reset (Aoi & Tsuchiya 2007)
                    yE(I.phi(i)) = P.th_TD - (j - 1)*pi;
                end
                % a reset can move the other leg's push-off command -> count as actuator work
                yE(I.Wact) = yE(I.Wact) + slip_energy(yE, W, P, opt) - E_before;
            case 'TO'                                    % trailing leg leaves the ground
                [~, j] = min(W(i).foot);
                W(i).contact(j) = false;  W(i).foot(j) = NaN;
                W(i).apexArmed = true;
                leg = j;
                % with a leg damper the spring is still loaded when F = 0; that energy is lost
                yE(I.El) = yE(I.El) + E_before - slip_energy(yE, W, P, opt);
            case 'APEX'
                leg = find(W(i).contact);
                W(i).apexArmed = false;
                if i == 1, nApex = nApex + 1; end
            otherwise                                    % failure
                leg = 0;  status = sprintf('%s (walker %d, t = %.3f s)', name, i, tE);  stop = true;
        end
        ev{end+1} = struct('t', tE, 'walker', i, 'type', name, 'leg', leg, ...
                           'foot', W(i).foot, 'y', yE);                         %#ok<AGROW>
    end
    if stop || nApex >= opt.nApexStop, break; end
    t0 = tE;  y0 = yE;
end

out.seg = seg;  out.ev = [ev{:}];  out.W = W;  out.status = status;  out.L0 = opt.L0;
out.opt = opt;  out.P = P;
end

function s = set_default(s, f, v)
if ~isfield(s, f) || isempty(s.(f)), s.(f) = v; end
end
