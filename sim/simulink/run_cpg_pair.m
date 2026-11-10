%% run_cpg_pair.m - one-click: load a preset, (re)build the model if needed, run, plot
%
% Change the preset and leg model here, or set them yourself and press Run in the model.

preset   = 'stage3_pair';   % 'stage1_pair' | 'stage2_single' | 'stage3_pair'
legmodel = 'footmass';          % 'damper' | 'footmass'   (Stage 2 and 3)
pairtype = 'different';       % 'identical' | 'different' (Lanini Pair 05 bodies)
lag0     = [];                % [] = start in step at an apex;  e.g. 90 = start with a 90 deg stride lag

cpg_pair_params;

% rebuild cpg_pair.slx when the block code is newer than the model
% (the MATLAB Function blocks hold a copy of sp_logic_block.m and sp_eom_block.m)
mdl = dir('cpg_pair.slx');
src = [dir('sp_logic_block.m'); dir('sp_eom_block.m'); dir('build_cpg_pair_model.m')];
if isempty(mdl) || any([src.datenum] > mdl.datenum)
    if bdIsLoaded('cpg_pair'), close_system('cpg_pair', 0); end
    build_cpg_pair_model;
end
open_system('cpg_pair');
out = sim('cpg_pair');
plot_cpg_pair_results;
