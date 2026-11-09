function f_v = CE_force_velocity_relation(vce_nrm, mp)
%
% Computes CE force-velocity relation.
%
% Args:
%   lce_nrm: The contractile element length normalized to the CE optimum length
%   mp: The model parameter structure
%
% Returns:
%   f_v: The normalized force of the force-velocity relation.

f_v_concentric = (vce_nrm + 1) ./ (1 - mp.K * vce_nrm);
f_v_eccentric = mp.N + (mp.N - 1) * (vce_nrm - 1) ./ (7.56 * mp.K * vce_nrm + 1);
mask = (vce_nrm <= 0);
f_v = f_v_concentric .* mask + f_v_eccentric .* ~mask;
