function dy = slip_rhs(~, y, W, P, opt)
% SLIP_RHS  Equations of motion of N spring-mass walkers, the object, the CPGs and
% the energy book-keeping states (layout: slip_idx).
%
% Each stance leg j of walker i pushes on the CoM along the leg (slip_leg):
%   F_j = k (lambda_j - l_j) - c dl_j/dt,     lambda_j = l0 + dl u(theta_j)
%   m xdd = sum_j F_j ex_j + F_arm,i          m zdd = sum_j F_j ez_j - m g
% Stage 1 (dl = c = 0, no CPG): exactly Geyer 2006 (one walker) / Lanini 2017 Eqs. 2-6 (pair).
%
% CPG of walker i (Owaki & Ishiguro 2017 Eq. 1, one oscillator per walker, leg j at phi + (j-1) pi):
%   dphi/dt = omega_i - sigma * sum_j N_j cos(theta_j),    N_j = vertical GRF of leg j [body weights]
% Phase reset at touchdown (Aoi & Tsuchiya 2007) is done in slip_sim.
%
% Energy book-keeping:
%   actuator power   sum_j Fs_j dlambda_j/dt       leg damper loss  sum_j c ld_j^2
%   arm damper loss  ba (dL1^2 + dL2^2)

nW = numel(W);  I = slip_idx(nW);
dy = zeros(size(y));
[Fw, Fo, Pdiss] = slip_arm(y, P, opt);
Pact = 0;  Pleg = 0;

for i = 1:nW
    q = y(4*i-3 : 4*i);  phi = y(I.phi(i));
    Fx = 0;  Fz = 0;  fb = 0;  Ls = {};
    for j = 1:2
        if W(i).contact(j)
            L  = slip_leg(q, phi, j, W(i).foot(j), P);
            Fx = Fx + L.F*L.ex;
            Fz = Fz + L.F*L.ez;
            fb = fb + (L.F*L.ez / (P.m*P.g)) * cos(L.th);
            Pleg = Pleg + P.c*L.ld^2;
            Ls{end+1} = L;                                                     %#ok<AGROW>
        end
    end
    if opt.coupled, Fx = Fx + Fw(i); end
    dy(4*i-3 : 4*i) = [q(3); q(4); Fx / P.m; Fz / P.m - P.g];

    if P.cpg
        dphi = P.omega(min(i, end)) - P.sigma*fb;
    else
        dphi = 0;
    end
    dy(I.phi(i)) = dphi;
    for n = 1:numel(Ls), Pact = Pact + Ls{n}.Fs * Ls{n}.dlam * dphi; end
end

dy(I.xo)   = y(I.xod);
dy(I.xod)  = opt.coupled * Fo / P.mt;
dy(I.Ea)   = Pdiss;
dy(I.Wact) = Pact;
dy(I.El)   = Pleg;
end
