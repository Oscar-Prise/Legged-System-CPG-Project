function S = sp_thin(R, dt)
% SP_THIN  Keep every sample of a sp_fixedstep_check result at a coarser step dt (for saving / videos).
k = 1:max(1, round(dt/(R.t(2) - R.t(1)))):numel(R.t);
S = R;
for f = {'t', 'pos', 'vel', 'phi', 'Eloss', 'legs', 'work', 'Ebal'}
    S.(f{1}) = R.(f{1})(k, :);
end
end
