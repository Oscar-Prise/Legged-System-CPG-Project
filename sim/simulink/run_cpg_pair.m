%% run_cpg_pair.m - one-click: load a preset, build the model if needed, run, plot
%
% Change the preset here, or set it yourself and press Run in the model.

preset = 'stage1_pair';     % 'stage1_pair' | 'stage2_single' | 'stage3_pair'

cpg_pair_params;
if ~exist('cpg_pair.slx', 'file'), build_cpg_pair_model; end
open_system('cpg_pair');
out = sim('cpg_pair');
plot_cpg_pair_results;
