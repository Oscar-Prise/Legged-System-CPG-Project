function f_l = PE_force_length_relation(lce_nrm, mp)
%
% Computes passive force-length relation.
%
% Args:
%   lce_nrm: The contractile element length normalized to the CE optimum length
%   mp: The model parameter structure
%
% Returns:
%   f_l: The normalized force of the force-length relation.

f_l = (lce_nrm > 1) .* ((lce_nrm - 1) / mp.w).^2;
