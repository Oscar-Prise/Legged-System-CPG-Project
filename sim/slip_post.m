function r = slip_post(out)
% SLIP_POST  Stitch the segments of slip_sim together and compute derived signals.
%
%   r.t, r.y                time and state
%   r.x, r.z, r.xd, r.zd    CoM signals, one column per walker
%   r.Fx, r.Fz              ground reaction force per walker (sum over stance legs)
%   r.FzLeg{i}              vertical GRF of leg 1 and leg 2 of walker i (two columns)
%   r.E                     mechanical energy per walker (kinetic + gravity + leg springs)
%   r.Etot                  E of all walkers + object kinetic + arm springs
%   r.Ediss                 energy dissipated in the arm dampers (integral of b dL^2)
%   r.Eleg, r.Wact          leg-damper loss and push-off actuator work (Stage 2)
%   r.phi                   CPG phase per walker
%   r.Ebal                  energy balance residual (should be ~0)
%   r.L, r.Farm             arm lengths and arm forces (two columns)

P = out.P;  opt = out.opt;
nW = numel(out.W);  I = slip_idx(nW);
t = [];  Y = [];  Fx = [];  Fz = [];  E = [];  Etot = [];  FzL = cell(1, nW);
for s = 1:numel(out.seg)
    S = out.seg{s};  n = numel(S.t);
    fx = zeros(n, nW);  fz = zeros(n, nW);  ew = zeros(n, nW);  et = zeros(n, 1);  fzl = cell(1, nW);
    for i = 1:nW, fzl{i} = zeros(n, 2); end
    for m = 1:n
        ym = S.y(m, :).';
        [et(m), ew(m, :)] = slip_energy(ym, S.W, P, opt);
        for i = 1:nW
            for j = 1:2
                if S.W(i).contact(j)
                    L = slip_leg(ym(4*i-3:4*i), ym(I.phi(i)), j, S.W(i).foot(j), P);
                    fx(m, i) = fx(m, i) + L.F*L.ex;
                    fz(m, i) = fz(m, i) + L.F*L.ez;
                    fzl{i}(m, j) = L.F*L.ez;
                end
            end
        end
    end
    if s > 1, k = 2:n; else, k = 1:n; end          % drop duplicated junction samples
    for i = 1:nW, FzL{i} = [FzL{i}; fzl{i}(k, :)]; end                                   %#ok<AGROW>
    t = [t; S.t(k)];  Y = [Y; S.y(k, :)];  Fx = [Fx; fx(k, :)];  Fz = [Fz; fz(k, :)];    %#ok<AGROW>
    E = [E; ew(k, :)];  Etot = [Etot; et(k)];                                            %#ok<AGROW>
end

r.t = t;  r.y = Y;
r.x = Y(:, 1:4:4*nW);  r.z = Y(:, 2:4:4*nW);  r.xd = Y(:, 3:4:4*nW);  r.zd = Y(:, 4:4:4*nW);
r.Fx = Fx;  r.Fz = Fz;  r.FzLeg = FzL;
r.E  = E;  r.Etot = Etot;
r.Ediss = Y(:, I.Ea);
r.phi   = Y(:, I.phi);
r.Wact  = Y(:, I.Wact);
r.Eleg  = Y(:, I.El);
% energy balance (must stay ~0):  Etot + arm loss + leg loss - actuator work - Etot(0)
r.Ebal  = r.Etot + r.Ediss + r.Eleg - r.Wact - (r.Etot(1) + r.Ediss(1) + r.Eleg(1) - r.Wact(1));

r.L = zeros(numel(t), 2);  r.Farm = zeros(numel(t), 2);
if opt.coupled
    for n = 1:numel(t)
        [~, ~, ~, L, Ld] = slip_arm(Y(n, :).', P, opt);
        r.L(n, :) = L;
        r.Farm(n, :) = (P.ka.*[1 1]).*(L - opt.L0) + (P.ba.*[1 1]).*Ld;
    end
end
end
