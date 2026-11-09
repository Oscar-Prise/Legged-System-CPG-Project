function vce_nrm = CE_inverse_force_velocity_relation(fv, mp)
%
% Computes CE velocity from inverted force-velocity relation.
%
% Args:
%   fv: The normalized force of the force-velocity relation.
%   mp: The model parameter structure
%
% Returns:
%   vce_nrm: The contractile element velocity normalized to the max
%   (absolute) contraction velocity. 
%

if fv <= 1  % concentric branch
  vce_nrm = (fv - 1) / (mp.K * fv + 1);
elseif (fv > 1) && (fv <= mp.N)  % eccentric branch
  vce_nrm = (fv - 1) / (mp.N - 1 - 7.56 * mp.K * (fv - mp.N));
else  % numerical eccentric overshoot
  vce_nrm = 1 + 0.01 * (fv - mp.N);  
end