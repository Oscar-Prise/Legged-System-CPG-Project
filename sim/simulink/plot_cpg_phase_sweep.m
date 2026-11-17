function plot_cpg_phase_sweep(files, outname)
% PLOT_CPG_PHASE_SWEEP  Maps of the initial-CPG-phase sweep: final offset, largest offset, settle time.
%   plot_cpg_phase_sweep({'../results/cpg_phase_sweep_damper_sig0_reset1.mat', ...}, '../results/fig_cpg_phase_sweep')
if ischar(files), files = {files}; end
nf = numel(files);
fig = figure('Position', [50 50 420*nf 820], 'Color', 'w');
for f = 1:nf
    L = load(files{f});  S = L.sweep;  ph = S.phis/pi;
    ttl = sprintf('%s, \\sigma = %g, %s', S.legmodel, S.sigma, ternary(S.reset > 0.5, 'reset at TD', 'free-running CPG'));
    for row = 1:3
        subplot(3, nf, (row - 1)*nf + f);
        switch row
            case 1, Z = S.offset_end;  cl = [0 90];  nm = 'final physical offset [deg] (0 = in step)';
            case 2, Z = S.maxoff;      cl = [0 90];  nm = 'largest offset during the run [deg]';
            case 3, Z = S.tsettle;     cl = [0 S.tEnd];  nm = 'time until within 5 deg of in step [s]';
        end
        imagesc(ph, ph, Z.');  axis xy;  caxis(cl);  colorbar;  hold on;
        [a, b] = find(S.fell);  plot(ph(a), ph(b), 'kx', 'MarkerSize', 12, 'LineWidth', 2);
        xlabel('\phi_1(0) / \pi');  ylabel('\phi_2(0) / \pi');
        title(sprintf('%s\n%s', ttl, nm), 'FontWeight', 'normal', 'FontSize', 9);
    end
end
sgtitle('Initial CPG phase sweep, both walkers started mechanically in step (x = pair fell)');
if nargin > 1 && ~isempty(outname), print(fig, '-dpng', '-r130', [outname '.png']); end
end
function s = ternary(c, a, b), if c, s = a; else, s = b; end, end
