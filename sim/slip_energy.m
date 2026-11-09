function [Etot, Ew] = slip_energy(y, W, P, opt)
% SLIP_ENERGY  Mechanical energy of the system in state y with contact data W.
%   Ew(i) = 1/2 m v^2 + m g z + sum_stance 1/2 k (lambda - l)^2      (walker i)
%   Etot  = sum Ew + 1/2 mt xod^2 + sum_arms 1/2 ka (L - L0)^2

nW = numel(W);  I = slip_idx(nW);  Ew = zeros(1, nW);
for i = 1:nW
    q = y(4*i-3 : 4*i);
    Ew(i) = 0.5*P.m*(q(3)^2 + q(4)^2) + P.m*P.g*q(2);
    for j = 1:2
        if W(i).contact(j)
            L = slip_leg(q, y(I.phi(i)), j, W(i).foot(j), P);
            Ew(i) = Ew(i) + 0.5*P.k*(L.lam - L.l)^2;
        end
    end
end
Etot = sum(Ew);
if opt.coupled
    [~, ~, ~, La] = slip_arm(y, P, opt);
    Etot = Etot + 0.5*P.mt*y(I.xod)^2 + sum(0.5*(P.ka.*[1 1]).*(La - opt.L0).^2);
end
end
