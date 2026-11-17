function P = sp_pair_poincare(mp, R, opts)
% SP_PAIR_POINCARE  Return (Poincare) map of the coupled pair, sectioned at walker 1's apex.
%
%   P = sp_pair_poincare(mp, R)         R = a run (sp_fixedstep_check) that ends near the gait
%   P = sp_pair_poincare(mp, R, opts)   opts.dt (map integration step, default 2e-5), opts.h (finite-
%                                       difference steps), opts.newton (iterations, default 4), opts.tol
%
% Section: walker 1's apex (dz1 = 0, falling) with the same contact configuration as the last apex
% of R (walking: leg 1 in stance; running: in flight, same leg landing next).  Reduced state at the
% section:
%   z1 dx1 (x2-x1) z2 dx2 dz2 (xo-x1) dxo phi1 phi2  + one entry per stance foot: (foot - body x)
%   + for the foot-mass model, per swing foot: (xf - x) zf vxf vzf
% The remaining discrete variables (rest lengths, take-off points/phases, event times) are taken
% from the reference section state and held.
% The map P(s) integrates one stride (next apex with the same configuration) and returns the state
% there.  The fixed point is refined by Newton with the numerical Jacobian J (central differences);
% the gait is stable when all |eig(J)| < 1.  (A CPG phase perturbation is erased at the next
% touchdown reset, so one eigenvalue per walker is 0.)
%
% P fields: s0, names, sfix, resid (|P(s)-s| after Newton), T (stride time), J, lam, V, cfg, ok

if nargin < 3, opts = struct(); end
if ~isfield(opts, 'dt'), opts.dt = 2e-5; end
if ~isfield(opts, 'newton'), opts.newton = 4; end
if ~isfield(opts, 'tol'), opts.tol = 2e-5; end
mp.dt = opts.dt;

% reference section state from the end of R
S = sections(R);
if isempty(S), error('no walker-1 apex found in R'); end
ref = S(end);
Tref = ref.t - S(end-2).t;                    % two apices per stride
P.cfg = ref.cfg;
[s, names] = pack(ref, mp);
P.s0 = s;  P.names = names;  n = numel(s);
isph = startsWith(names, 'phi');
if ~isfield(opts, 'h'), opts.h = 5e-3*ones(n, 1);  opts.h(isph) = 2e-2; end

pm = @(sv) pmap(sv, ref, mp, Tref);
fprintf('sp_pair_poincare: %d-D section state [%s], stride %.3f s\n', n, strjoin(names, ' '), Tref);

% Newton refinement of the fixed point
P.ok = true;  best = s;  rbest = Inf;
for it = 1:opts.newton
    [s1, T, ok] = pm(s);
    if ~ok, warning('map evaluation failed at Newton step %d, keeping the best state', it);  break; end
    r = wrapd(s1 - s, isph);
    fprintf('  Newton %d: |P(s)-s| = %.2e, T = %.4f s\n', it - 1, norm(r), T);
    if norm(r) < rbest, best = s;  rbest = norm(r); end
    if norm(r) < opts.tol, break; end
    if norm(r) > 3*rbest, break; end            % diverging (map noise from event quantisation)
    J = jac(pm, s, opts.h, isph);
    s = s - (J - eye(n)) \ r;
    s(isph) = mod(s(isph), 2*pi);
end
s = best;
P.sfix = s;
[s1, T, ok] = pm(s);
if ok, P.resid = norm(wrapd(s1 - s, isph)); else, P.resid = NaN; P.ok = false; end
P.T = T;
P.J = jac(pm, s, opts.h, isph);
[P.V, D] = eig(P.J);  P.lam = diag(D);
[~, o] = sort(abs(P.lam), 'descend');  P.lam = P.lam(o);  P.V = P.V(:, o);
fprintf('  fixed point residual %.2e, |eig| = %s\n', P.resid, mat2str(abs(P.lam.'), 3));
end

% ------------------------------------------------------------------------------
function J = jac(pm, s, h, isph)
n = numel(s);  J = zeros(n);
for j = 1:n
    sp = s;  sp(j) = sp(j) + h(j);  sm = s;  sm(j) = sm(j) - h(j);
    [a, ~, oka] = pm(sp);  [b, ~, okb] = pm(sm);
    if oka && okb
        J(:, j) = wrapd(a - b, isph)/(2*h(j));
    else                                        % one-sided difference through the fixed point
        [c, ~, okc] = pm(s);
        if oka && okc, J(:, j) = wrapd(a - c, isph)/h(j);
        elseif okb && okc, J(:, j) = wrapd(c - b, isph)/h(j);
        else, error('map evaluation failed while building the Jacobian (coordinate %d)', j); end
        warning('sp_pair_poincare: one-sided difference for coordinate %d', j);
    end
end
end

function d = wrapd(d, isph)
d(isph) = mod(d(isph) + pi, 2*pi) - pi;
end

function [s1, T, ok] = pmap(s, ref, mp, Tref)
[ic, mpi] = unpack(s, ref, mp);
R = sp_fixedstep_check(1.7*Tref, mpi, ic);
s1 = nan(size(s));  T = NaN;  ok = false;
if R.stop > 0, return; end
S = sections(R);
S = S([S.t] > 0.4*Tref & arrayfun(@(q) isequal(q.cfg, ref.cfg), S));
if isempty(S), return; end
[~, k] = min(abs([S.t] - Tref));
[s1, nm] = pack(S(k), mp);
if numel(s1) ~= numel(s), s1 = nan(size(s)); return; end   %#ok<NASGU>
T = S(k).t;  ok = true;
end

function S = sections(R)
% walker-1 apices (dz1 falls through 0), state interpolated, discrete state from the sample before
dz = R.vel(:, 2);
k = find(dz(1:end-1) > 0 & dz(2:end) <= 0);
S = struct('t', {}, 'q', {}, 'legs', {}, 'state', {}, 'phi', {}, 'cfg', {});
for m = 1:numel(k)
    i = k(m);  a = dz(i)/(dz(i) - dz(i+1));
    q = (1 - a)*[R.pos(i, :), R.vel(i, :)] + a*[R.pos(i+1, :), R.vel(i+1, :)];
    dph = mod(R.phi(i+1, :) - R.phi(i, :) + pi, 2*pi) - pi;
    S(end+1).t = (1 - a)*R.t(i) + a*R.t(i+1);          %#ok<AGROW>
    S(end).q = q.';  S(end).legs = R.legs(i, :);  S(end).state = R.state(i, :);
    S(end).phi = mod(R.phi(i, :) + a*dph, 2*pi);
    c = R.legs(i, 1:4) > 0.5;  nxt = 0;
    if ~any(c(1:2)), nxt = 1;  if R.legs(i, 34) < R.legs(i, 33), nxt = 2; end, end
    S(end).cfg = [c, nxt];
end
end

function [s, names] = pack(sec, mp)
pos = sec.q(1:5);  vel = sec.q(6:10);  legs = sec.legs;  st = sec.state;
s = [pos(2); vel(1); pos(3) - pos(1); pos(4); vel(3); vel(4); pos(5) - pos(1); vel(5); sec.phi(1); sec.phi(2)];
names = {'z1', 'dx1', 'x2-x1', 'z2', 'dx2', 'dz2', 'xo-x1', 'dxo', 'phi1', 'phi2'};
for i = 1:2
    for j = 1:2
        n = 2*(i-1) + j;  b = 25*(i-1);
        if legs(n) > 0.5
            s(end+1) = legs(4+n) - pos(2*i-1);                     %#ok<AGROW>
            names{end+1} = sprintf('footW%dL%d', i, j);            %#ok<AGROW>
        elseif mp.legmodel > 1.5                                   % swing foot of the foot model
            s(end+1:end+4) = [st(b+8+j) - pos(2*i-1); st(b+10+j); st(b+12+j); st(b+14+j)];   %#ok<AGROW>
            names(end+1:end+4) = {sprintf('swW%dL%dx', i, j), sprintf('swW%dL%dz', i, j), ...
                                  sprintf('swW%dL%dvx', i, j), sprintf('swW%dL%dvz', i, j)};   %#ok<AGROW>
        end
    end
end
end

function [ic, mpi] = unpack(s, ref, mp)
pos = [0; s(1); s(3); s(4); s(7)];  vel = [s(2); 0; s(5); s(6); s(8)];
phi = [s(9); s(10)];  k = 10;
init = zeros(1, 50);  f0 = [0 0];
for i = 1:2
    b = 25*(i-1);  S = ref.state(b + (1:25));
    x = pos(2*i-1);  z = pos(2*i);  xr = ref.q(2*i-1);  l0 = mp.l0(i);  a0 = mp.a0(i);
    c = S(1:2) > 0.5;
    f = [0 0];  xf = [0 0];  zf = [0 0];  vxf = [0 0];  vzf = [0 0];
    for j = 1:2
        if c(j)
            k = k + 1;  f(j) = x + s(k);  xf(j) = f(j);  zf(j) = 0;
        elseif mp.legmodel > 1.5
            xf(j) = x + s(k+1);  zf(j) = s(k+2);  vxf(j) = s(k+3);  vzf(j) = s(k+4);  k = k + 4;
        else
            xf(j) = x + l0*cos(a0);  zf(j) = z - l0*sin(a0);
        end
    end
    lb = S(5:6);  lamp = S(7:8);
    xTO = S(17:18) - xr + x;  thTO = S(19:20);
    tTD = S(21:22) - ref.t;  tTO = S(23:24) - ref.t;
    init(b + (1:25)) = [double(c), f, lb, lamp, xf, zf, vxf, vzf, xTO, thTO, tTD, tTO, phi(i)];
    f0(i) = f(1);
end
mpi = mp;  mpi.init = init;  mpi.init_use = [1 1];  mpi.foot0 = f0;  mpi.phi0 = phi.';
ic.pos = pos;  ic.vel = vel;  ic.foot = f0.';  ic.phi = phi;
end
