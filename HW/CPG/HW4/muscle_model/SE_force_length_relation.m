function f_l = SE_force_length_relation(lse_nrm, mp)
%
% Computes force-length relation of the series elasticity.
%
% Args:
%   lse_nrm: The series elasticity length normalized to its slack length
%   mp: The model parameter structure
%
% Returns:
%   f_l: The normalized force of the force-length relation.

f_l = (lse_nrm > 1) .* ((lse_nrm - 1) / mp.eref).^2;
