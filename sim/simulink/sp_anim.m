function sp_anim(block)
%
% Level-2 MATLAB S-function: side-view animation of the paired spring-mass walkers
% (same structure as the course animation m07_anim.m, H. Geyer).
%
% Inputs:
% port 1: positions [x1 z1 x2 z2 xo]          (port width: 5)
% port 2: legs from the gait logic            (port width: 12)
%         [c11 c12 c21 c22, f11 f12 f21 f22, lam11 lam12 lam21 lam22]
%
% Dialog parameters:
% Parameter 1: model parameter structure mp
% Parameter 2: animation real-time factor (1 = real time, 0.5 = half speed)
%
% mp.movie = 1 also writes cpg_pair.mp4, mp.gif = 1 writes cpg_pair.gif
% (into the current folder; rename the file before the next run).
%

persistent hFig TimingObj AnimObj MovieObj

% view window
ViewXRange = 5;    % [m]

setup(block);


% ##############################################################################
function setup(block)

  block.SimStateCompliance = 'DefaultSimState';

  block.NumInputPorts  = 2;
  block.NumOutputPorts = 0;

  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;

  block.InputPort(1).Dimensions  = 5;
  block.InputPort(1).DatatypeID  = 0;
  block.InputPort(1).Complexity  = 'Real';
  block.InputPort(1).DirectFeedthrough = true;

  block.InputPort(2).Dimensions  = 12;
  block.InputPort(2).DatatypeID  = 0;
  block.InputPort(2).Complexity  = 'Real';
  block.InputPort(2).DirectFeedthrough = true;

  block.NumDialogPrms = 2;      % mp, real-time factor

  % one frame every mp.dt_anim seconds of simulation time
  block.SampleTimes = [block.DialogPrm(1).Data.dt_anim 0];

  block.RegBlockMethod('Start',     @Start);
  block.RegBlockMethod('Outputs',   @Outputs);   % required
  block.RegBlockMethod('Update',    @Update);
  block.RegBlockMethod('Terminate', @Terminate); % required
end


% ##############################################################################
function Start(block)

  mp = block.DialogPrm(1).Data;
  NameStr = 'CPG-driven paired spring-mass walkers';

  hFig = findobj(0, 'type', 'figure', 'name', NameStr);
  if isempty(hFig)
    hFig = figure('Name', NameStr, 'NumberTitle', 'off', ...
                  'Color', [1 1 1], 'Position', [100 100 1100 420]);
  end
  set(0, 'currentfigure', hFig);
  clf(hFig);
  ax = axes('Parent', hFig, 'FontSize', 9, 'Position', [0.05 0.12 0.92 0.8]);
  hold(ax, 'on');  box(ax, 'on');
  daspect(ax, [1 1 1]);
  xlabel(ax, 'x [m]');  ylabel(ax, 'height [m]');

  % ground with tick marks so forward motion is visible
  plot(ax, [-10 500], [0 0], 'k-', 'LineWidth', 1.5);
  gx = -10:0.5:500;
  tx = [gx; gx - 0.1; NaN(size(gx))];  tz = [zeros(size(gx)); -0.06*ones(size(gx)); NaN(size(gx))];
  plot(ax, tx(:), tz(:), '-', 'Color', [0.6 0.6 0.6]);

  col = [0 0.45 0.74; 0.85 0.33 0.10];       % walker 1 blue, walker 2 orange
  AnimObj = [];
  AnimObj.col = col;
  for i = 1:2
    vis = 'on';  if i > mp.nW, vis = 'off'; end
    for j = 1:2
      lw = 3.5;  if j == 2, lw = 2; end      % leg 1 thick, leg 2 thin
      AnimObj.Leg(i,j) = plot(ax, [0 0], [0 0], '-', 'Color', col(i,:), 'LineWidth', lw, 'Visible', vis);
    end
    AnimObj.Body(i) = plot(ax, 0, 0, 'o', 'MarkerSize', 22, 'MarkerFaceColor', col(i,:), ...
                           'MarkerEdgeColor', 'k', 'Visible', vis);
  end

  % object and arms (drawn at a fixed height; the model has horizontal arms only)
  vis = 'off';  if mp.coupled > 0.5 && mp.nW > 1.5, vis = 'on'; end
  AnimObj.hObj = 0.85;  AnimObj.wObj = 0.5;
  AnimObj.Arm(1) = plot(ax, [0 0], [0 0], 'k-', 'LineWidth', 1.5, 'Visible', vis);
  AnimObj.Arm(2) = plot(ax, [0 0], [0 0], 'k-', 'LineWidth', 1.5, 'Visible', vis);
  AnimObj.Obj = patch(ax, [0 0 0 0], [0 0 0 0], [0.6 0.6 0.6], 'EdgeColor', 'k', 'Visible', vis);

  AnimObj.Text = text(ax, 0, 1.3, '', 'FontSize', 10);
  AnimObj.prevC = [1 0; 1 0];
  AnimObj.ax = ax;
  ylim(ax, [-0.1 1.4]);
  xlim(ax, [-1, ViewXRange - 1]);

  % movie / gif
  AnimObj.gifFirst = true;
  if mp.movie > 0.5
    MovieObj = VideoWriter('cpg_pair.mp4', 'MPEG-4');
    MovieObj.FrameRate = round(1/mp.dt_anim);
    open(MovieObj);
  end

  % timing: hold the animation to rtf x real time
  TimingObj = [];
  TimingObj.t0 = tic;
  TimingObj.RealTimeFactor = block.DialogPrm(2).Data;
end


% ##############################################################################
function Outputs(block)
  % nothing to do here
end


% ##############################################################################
function Update(block)

  if isempty(hFig) || ~ishandle(hFig), return; end     % figure was closed
  mp  = block.DialogPrm(1).Data;
  pos = block.InputPort(1).Data;
  legs = block.InputPort(2).Data;
  ax = AnimObj.ax;

  for i = 1:mp.nW
    x = pos(2*i-1);  z = pos(2*i);
    set(AnimObj.Body(i), 'XData', x, 'YData', z);
    for j = 1:2
      n = 2*(i-1) + j;
      if legs(n) > 0.5                                    % stance: CoM to foot
        set(AnimObj.Leg(i,j), 'XData', [x legs(4+n)], 'YData', [z 0], 'LineStyle', '-');
        if AnimObj.prevC(i,j) < 0.5                       % touchdown: footprint
          plot(ax, legs(4+n), 0, '.', 'Color', AnimObj.col(i,:), 'MarkerSize', 14);
        end
      else                                                % swing: held at the angle of attack
        set(AnimObj.Leg(i,j), 'XData', [x x + mp.l0*cos(mp.a0)], ...
            'YData', [z z - mp.l0*sin(mp.a0)], 'LineStyle', ':');
      end
      AnimObj.prevC(i,j) = legs(n);
    end
  end

  if mp.coupled > 0.5 && mp.nW > 1.5
    xo = pos(5);  h = AnimObj.hObj;  w = AnimObj.wObj;
    set(AnimObj.Obj, 'XData', xo + w/2*[-1 1 1 -1], 'YData', h + 0.05*[-1 -1 1 1]);
    set(AnimObj.Arm(1), 'XData', [pos(1) xo - w/2], 'YData', [pos(2) h]);
    set(AnimObj.Arm(2), 'XData', [xo + w/2 pos(3)], 'YData', [h pos(4)]);
  end

  % camera follows the walkers
  if mp.nW > 1.5, xc = 0.5*(pos(1) + pos(3)); else, xc = pos(1); end
  xl = xc + ViewXRange*[-0.5 0.5];
  set(ax, 'XLim', xl);
  set(AnimObj.Text, 'Position', [xl(1) + 0.1, 1.3, 0], ...
      'String', sprintf('t = %.2f s', block.CurrentTime));

  % real-time pacing
  target = block.CurrentTime / TimingObj.RealTimeFactor;
  lag = target - toc(TimingObj.t0);
  if lag > 0, pause(lag); end
  drawnow limitrate

  if mp.movie > 0.5
    writeVideo(MovieObj, getframe(hFig));
  end
  if mp.gif > 0.5                                         % animated gif, real-time playback
    [A, map] = rgb2ind(frame2im(getframe(hFig)), 128);
    if AnimObj.gifFirst
      imwrite(A, map, 'cpg_pair.gif', 'gif', 'LoopCount', Inf, 'DelayTime', mp.dt_anim);
      AnimObj.gifFirst = false;
    else
      imwrite(A, map, 'cpg_pair.gif', 'gif', 'WriteMode', 'append', 'DelayTime', mp.dt_anim);
    end
  end
end


% ##############################################################################
function Terminate(block)
  if block.DialogPrm(1).Data.movie > 0.5 && ~isempty(MovieObj)
    close(MovieObj);
  end
end

end
