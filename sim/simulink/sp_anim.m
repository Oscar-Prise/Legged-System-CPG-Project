function sp_anim(block)
%
% Level-2 MATLAB S-function: animation of the paired spring-mass walkers
% (same block interface as the course animation m07_anim.m, H. Geyer).
%
% Top panel:    side view drawn like Lanini 2017 Fig 6: walker 1 green, walker 2 red,
%               object = black point mass m_t, arms = spring || damper.  Legs:
%                 passive (Lanini)    ->  spring leg; massless swing leg not drawn (as in Fig 6)
%                 leg damper model    ->  spring || damper leg; swing leg not drawn
%                 foot-mass model     ->  spring leg, swing leg drawn with its foot mass (dot)
%               Leg colours: walker 1 right leg (leg 1) green, left leg (leg 2) blue;
%               walker 2 right leg red, left leg purple.  A gold glow behind a stance leg =
%               CPG push-off on; the clock above each walker is its CPG phase.
% Bottom panel: footfall diagram (bar = foot on the ground), last 4 s, with the stride
%               phase lag of walker 2 vs walker 1 (leg-1 touchdowns) and the nearest of
%               Lanini 2017's reference lags (pace ~0, trot ~180, diagonal sequence ~225 deg).
%
% Inputs:
% port 1: positions [x1 z1 x2 z2 xo]          (width 5)
% port 2: legs from the gait logic            (width 36, see sp_logic_block.m)
% port 3: CPG phases [phi1 phi2]              (width 2)
%
% Dialog parameters:
% Parameter 1: model parameter structure mp
% Parameter 2: animation real-time factor (1 = real time, 0.5 = half speed)
%
% mp.movie = 1 also writes cpg_pair.mp4, mp.gif = 1 writes cpg_pair.gif
% (into the current folder; rename the file before the next run).
%
% The drawing state is kept in the figure (setappdata), not in persistent variables.
%
setup(block);
end


% ##############################################################################
function setup(block)

  block.SimStateCompliance = 'DefaultSimState';

  block.NumInputPorts  = 3;
  block.NumOutputPorts = 0;

  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;

  w = [5 36 2];
  for p = 1:3
    block.InputPort(p).Dimensions  = w(p);
    block.InputPort(p).DatatypeID  = 0;
    block.InputPort(p).Complexity  = 'Real';
    block.InputPort(p).DirectFeedthrough = true;
  end

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
  hFig = findobj(0, 'type', 'figure', 'name', fig_name());
  if isempty(hFig)
    hFig = figure('Name', fig_name(), 'NumberTitle', 'off', ...
                  'Color', [1 1 1], 'Position', [80 60 1100 680]);
  end
  hFig = hFig(1);
  set(0, 'currentfigure', hFig);
  clf(hFig);

  A = struct();
  A.ViewXRange = 5;                                     % [m] width of the side view
  A.Window = 4;                                         % [s] width of the footfall diagram
  A.col  = [0.13 0.55 0.13; 0.85 0.10 0.10];           % walker 1 green, walker 2 red (Lanini Fig 6)
  A.legcol = cat(3, [0.13 0.55 0.13; 0.85 0.10 0.10], ...  % (walker, leg, rgb): right legs
                    [0.10 0.40 0.90; 0.55 0.20 0.70]);     %                     left legs
  A.legcol = permute(A.legcol, [1 3 2]);
  A.push = [1.00 0.80 0.25];                            % push-off on: gold glow
  if mp.legmodel > 1.5, A.style = 3; elseif mp.c(1) > 0, A.style = 2; else, A.style = 1; end
  A.showCPG = mp.cpg > 0.5 || mp.legmodel > 1.5;
  A.pair = mp.coupled > 0.5 && mp.nW > 1.5;

  % ---------------- top: side view
  ax = axes('Parent', hFig, 'FontSize', 9, 'Position', [0.06 0.40 0.91 0.52]);
  hold(ax, 'on');  box(ax, 'on');
  daspect(ax, [1 1 1]);
  xlabel(ax, 'x [m]');  ylabel(ax, 'height [m]');
  plot(ax, [-10 500], [0 0], 'k-', 'LineWidth', 1.5);
  gx = -10:0.5:500;
  tx = [gx; gx - 0.1; NaN(size(gx))];  tz = [zeros(size(gx)); -0.06*ones(size(gx)); NaN(size(gx))];
  plot(ax, tx(:), tz(:), '-', 'Color', [0.6 0.6 0.6]);

  for i = 1:2
    vis = 'on';  if i > mp.nW, vis = 'off'; end
    for j = 1:2                                         % glow first, so it is drawn behind
      A.Glow(i,j) = plot(ax, [0 0], [0 0], '-', 'Color', A.push, 'LineWidth', 10, 'Visible', 'off');
    end
    for j = 1:2
      lc = squeeze(A.legcol(i,j,:)).';
      A.Leg(i,j)  = plot(ax, [0 0], [0 0], '-', 'Color', lc, 'LineWidth', 2, 'Visible', vis);
      A.Foot(i,j) = plot(ax, 0, 0, 'o', 'MarkerSize', 7, 'MarkerFaceColor', lc, ...
                         'MarkerEdgeColor', 'k', 'Visible', 'off');
    end
    ms = 22*(mp.m(i)/80)^(1/3);                         % marker size grows with body mass
    A.Body(i) = plot(ax, 0, 0, 'o', 'MarkerSize', ms, 'MarkerFaceColor', A.col(i,:), ...
                     'MarkerEdgeColor', 'k', 'Visible', vis);
    dvis = 'off';  if A.showCPG && i <= mp.nW, dvis = 'on'; end
    A.Dial(i) = plot(ax, 0, 0, '-', 'Color', [0.3 0.3 0.3], 'LineWidth', 1, 'Visible', dvis);
    A.Hand(i) = plot(ax, [0 0], [0 0], '-', 'Color', A.push, 'LineWidth', 2, 'Visible', dvis);
    A.DialTxt(i) = text(0, 0, 'CPG', 'Parent', ax, 'FontSize', 8, 'Color', [0.3 0.3 0.3], ...
                        'Visible', dvis);
  end

  % object (point mass m_t) and arms (spring || damper, Lanini Fig 6).  The model's arms are
  % horizontal; the object is drawn at the mean CoM height of the two walkers.
  vis = 'off';  if A.pair, vis = 'on'; end
  A.rObj = 0.07;                                        % [m] drawn radius of the object
  A.Arm(1) = plot(ax, [0 0], [0 0], '-', 'Color', A.col(1,:), 'LineWidth', 1.3, 'Visible', vis);
  A.Arm(2) = plot(ax, [0 0], [0 0], '-', 'Color', A.col(2,:), 'LineWidth', 1.3, 'Visible', vis);
  ac = linspace(0, 2*pi, 30).';
  A.circ = [cos(ac), sin(ac)];
  A.Obj = patch(A.rObj*A.circ(:,1), A.rObj*A.circ(:,2), [0 0 0], 'Parent', ax, 'EdgeColor', 'k', 'Visible', vis);

  A.Text = text(0, 1.42, '', 'Parent', ax, 'FontSize', 10);
  % leg colour key (top right)
  key = {'W1 right', 'W1 left', 'W2 right', 'W2 left'};
  for n = 1:2*mp.nW
    i = ceil(n/2);  j = n - 2*(i-1);
    text(0.99, 0.91 - 0.07*(n-1), key{n}, 'Parent', ax, 'Units', 'normalized', ...
         'HorizontalAlignment', 'right', 'FontSize', 9, 'FontWeight', 'bold', ...
         'Color', squeeze(A.legcol(i,j,:)).');
  end
  title(ax, model_label(mp), 'FontSize', 11, 'FontWeight', 'normal');
  ylim(ax, [-0.1 1.55]);
  xlim(ax, [-1, A.ViewXRange - 1]);
  A.ax = ax;

  % ---------------- bottom: footfall diagram
  ax2 = axes('Parent', hFig, 'FontSize', 9, 'Position', [0.06 0.07 0.91 0.22]);
  hold(ax2, 'on');  box(ax2, 'on');
  nr = 2*mp.nW;
  lab = cell(1, nr);
  for i = 1:mp.nW
    for j = 1:2
      A.row(i,j) = nr + 1 - (2*(i-1) + j);              % walker 1 leg 1 on top
      lr = {'right', 'left'};
      lab{A.row(i,j)} = sprintf('W%d %s', i, lr{j});
    end
  end
  set(ax2, 'YTick', 1:nr, 'YTickLabel', lab, 'YLim', [0.4 nr + 0.6], 'XLim', [-A.Window 0]);
  xlabel(ax2, 'time [s]');
  A.ax2 = ax2;
  A.FFTitle = title(ax2, 'Footfall  (bar = foot on the ground)', 'FontSize', 10, 'FontWeight', 'normal');
  A.bar = cell(2, 2);                                   % open stance bar per leg
  A.lastTD = -Inf(1, 2);                                % last leg-1 touchdown handled
  A.td1 = {zeros(1, 0), zeros(1, 0)};                   % leg-1 touchdown times per walker
  A.lagStr = '';

  % movie / gif
  A.gifFirst = true;
  A.Movie = [];
  if mp.movie > 0.5
    A.Movie = VideoWriter('cpg_pair.mp4', 'MPEG-4');
    A.Movie.FrameRate = round(1/mp.dt_anim);
    open(A.Movie);
  end

  % timing: hold the animation to rtf x real time
  A.t0 = tic;
  A.RealTimeFactor = block.DialogPrm(2).Data;

  setappdata(hFig, 'A', A);
end


% ##############################################################################
function Outputs(block)
  % nothing to do here
end


% ##############################################################################
function Update(block)

  hFig = findobj(0, 'type', 'figure', 'name', fig_name());
  if isempty(hFig), return; end                         % figure was closed
  hFig = hFig(1);
  A = getappdata(hFig, 'A');
  if isempty(A), return; end
  mp   = block.DialogPrm(1).Data;
  pos  = block.InputPort(1).Data;
  legs = block.InputPort(2).Data;
  phi  = block.InputPort(3).Data;
  t    = block.CurrentTime;

  % ---------------- side view
  for i = 1:mp.nW
    x = pos(2*i-1);  z = pos(2*i);
    set(A.Body(i), 'XData', x, 'YData', z);
    for j = 1:2
      n = 2*(i-1) + j;
      stance = legs(n) > 0.5;
      if stance, fx = legs(4+n); fz = 0; else, fx = legs(16+n); fz = legs(20+n); end
      col = squeeze(A.legcol(i,j,:)).';
      if ~stance, col = 0.55*col + 0.45; end            % swing leg lighter
      if stance || A.style == 3
        if A.style == 2, kind = 2; else, kind = 1; end  % spring || damper, or spring
        [X, Y] = link_shape([x z], [fx fz], kind, mp.l0(i), 0.05*mp.l0(i));
        set(A.Leg(i,j), 'XData', X, 'YData', Y, 'Color', col, 'LineWidth', 2, 'Visible', 'on');
      else                                              % massless swing leg: not drawn (Fig 6)
        set(A.Leg(i,j), 'Visible', 'off');
      end
      if stance && legs(12+n) > 0.05                    % CPG push-off on: gold glow
        set(A.Glow(i,j), 'XData', [x fx], 'YData', [z fz], 'Visible', 'on');
      else
        set(A.Glow(i,j), 'Visible', 'off');
      end
      if A.style == 3 && ~stance
        set(A.Foot(i,j), 'XData', fx, 'YData', fz, 'Visible', 'on');
      else
        set(A.Foot(i,j), 'Visible', 'off');
      end
    end
    if A.showCPG                                        % CPG phase clock above the walker
      cx = x;  cz = z + 0.36;  r = 0.09;  a = linspace(0, 2*pi, 40);
      set(A.Dial(i), 'XData', cx + r*cos(a), 'YData', cz + r*sin(a));
      set(A.Hand(i), 'XData', [cx, cx + r*cos(phi(i))], 'YData', [cz, cz + r*sin(phi(i))]);
      set(A.DialTxt(i), 'Position', [cx + 0.12, cz, 0]);
    end
  end

  if A.pair
    xo = pos(5);  zo = 0.5*(pos(2) + pos(4));  r = A.rObj;
    set(A.Obj, 'XData', xo + r*A.circ(:,1), 'YData', zo + r*A.circ(:,2));
    rb = 0.09;                                          % body marker radius (approx.)
    [X, Y] = link_shape([pos(1) + rb, pos(2)], [xo - r, zo], 2, mp.L0(1) - r - rb, 0.04);
    set(A.Arm(1), 'XData', X, 'YData', Y);
    [X, Y] = link_shape([xo + r, zo], [pos(3) - rb, pos(4)], 2, mp.L0(2) - r - rb, 0.04);
    set(A.Arm(2), 'XData', X, 'YData', Y);
  end

  % camera follows the walkers
  if mp.nW > 1.5, xc = 0.5*(pos(1) + pos(3)); else, xc = pos(1); end
  xl = xc + A.ViewXRange*[-0.5 0.5];
  set(A.ax, 'XLim', xl);
  set(A.Text, 'Position', [xl(1) + 0.1, 1.42, 0], 'String', sprintf('t = %.2f s', t));

  % ---------------- footfall diagram
  for i = 1:mp.nW
    for j = 1:2
      n = 2*(i-1) + j;
      tTD = legs(28+n);  tTO = legs(32+n);
      y = A.row(i,j) + 0.32*[-1; -1; 1; 1];          % patch data as columns
      fc = squeeze(A.legcol(i,j,:)).';
      if legs(n) > 0.5
        if isempty(A.bar{i,j})
          A.bar{i,j} = patch([tTD; t; t; tTD], y, fc, 'Parent', A.ax2, 'EdgeColor', 'none');
        else
          set(A.bar{i,j}, 'XData', [tTD; t; t; tTD]);
        end
      elseif ~isempty(A.bar{i,j})
        set(A.bar{i,j}, 'XData', [tTD; tTO; tTO; tTD]);    % stance ended: close the bar
        A.bar{i,j} = [];
      end
      if j == 1 && tTD > A.lastTD(i) + 1e-9 && tTD > 0  % new leg-1 touchdown
        A.lastTD(i) = tTD;
        A.td1{i}(end+1) = tTD;
        if i == 2, A.lagStr = lag_string(A.td1{1}, tTD); end
        if i == 1 && mp.nW > 1.5 && ~isempty(A.td1{2})
          A.lagStr = lag_string(A.td1{1}, A.td1{2}(end));
        end
      end
    end
  end
  set(A.ax2, 'XLim', t + [-A.Window 0.1]);
  if mp.nW > 1.5 && ~isempty(A.lagStr)
    set(A.FFTitle, 'String', ['Footfall (bar = foot on the ground)     ' A.lagStr]);
  end

  setappdata(hFig, 'A', A);

  % real-time pacing
  target = t / A.RealTimeFactor;
  lag = target - toc(A.t0);
  if lag > 0, pause(lag); end
  drawnow limitrate

  if mp.movie > 0.5 && ~isempty(A.Movie)
    writeVideo(A.Movie, getframe(hFig));
  end
  if mp.gif > 0.5                                       % animated gif, real-time playback
    [Im, map] = rgb2ind(frame2im(getframe(hFig)), 128);
    if A.gifFirst
      imwrite(Im, map, 'cpg_pair.gif', 'gif', 'LoopCount', Inf, 'DelayTime', mp.dt_anim);
      A.gifFirst = false;
      setappdata(hFig, 'A', A);
    else
      imwrite(Im, map, 'cpg_pair.gif', 'gif', 'WriteMode', 'append', 'DelayTime', mp.dt_anim);
    end
  end
end


% ##############################################################################
function Terminate(block)
  hFig = findobj(0, 'type', 'figure', 'name', fig_name());
  if isempty(hFig), return; end
  A = getappdata(hFig(1), 'A');
  if ~isempty(A) && ~isempty(A.Movie)
    close(A.Movie);
  end
end


% ##############################################################################
function s = fig_name()
  s = 'CPG-driven paired spring-mass walkers';
end

function s = model_label(mp)
% one-line description of what is simulated
  if mp.nW > 1.5
    if abs(mp.m(1) - mp.m(2)) > 1e-9 || abs(mp.l0(1) - mp.l0(2)) > 1e-9
      w = sprintf('2 different walkers (%g kg/%g m, %g kg/%g m)', mp.m(1), mp.l0(1), mp.m(2), mp.l0(2));
    else
      w = '2 identical walkers';
    end
    if isfield(mp, 'lag0') && ~isnan(mp.lag0), w = sprintf('%s, start lag %g deg', w, mp.lag0); end
  else
    w = '1 walker';
  end
  if mp.legmodel > 1.5
    leg = sprintf('spring legs + feet (dot), CPG swing');
  elseif mp.c(1) > 0
    leg = sprintf('spring || damper legs');
  else
    leg = 'passive spring legs (Lanini 2017)';
  end
  if mp.cpg > 0.5, cp = 'CPG (gold = push-off)'; else, cp = 'passive, no CPG'; end
  s = [w '  |  ' leg '  |  ' cp];
end

function [X, Y] = link_shape(S, E, kind, N, w)
% polyline of a massless link from point S to point E (row vectors)
%   kind 1: spring (zigzag)
%   kind 2: spring || damper (spring on one side, dashpot on the other, end bars)
% N = nominal (rest) length, sets the end rods and the dashpot proportions; w = half width
  L = norm(E - S);  e = (E - S)/L;  nv = [-e(2), e(1)];
  if kind == 1
    P = zigzag(S, E, 0.12*N, w);
  else
    r  = 0.10*N;  off = 1.3*w;
    S1 = S + r*e;  E1 = E - r*e;
    P = [S; S1; NaN NaN; S1 + off*nv; S1 - off*nv; NaN NaN; E; E1; NaN NaN; ...
         E1 + off*nv; E1 - off*nv; NaN NaN; ...
         zigzag(S1 + off*nv, E1 + off*nv, 0.08*N, 0.7*w); NaN NaN; ...
         dashpot(S1 - off*nv, E1 - off*nv, N - 2*r, 0.8*w)];
  end
  X = P(:, 1).';  Y = P(:, 2).';
end

function P = zigzag(S, E, lead, w)
% zigzag spring with 8 coils between straight leads
  L = norm(E - S);  e = (E - S)/L;  nv = [-e(2), e(1)];
  lead = min(lead, 0.3*L);
  nc = 8;
  sp = linspace(lead, L - lead, 2*nc + 1).';
  o  = w*[0; repmat([1; -1], nc - 1, 1); 1; 0];
  o  = o(1:numel(sp));  o(end) = 0;
  P = [S; S + sp*e + o*nv; E];
end

function P = dashpot(S, E, N, w)
% dashpot: rod from S to a piston plate, open cylinder on a rod from E; the plate slides
% in the cylinder as the length changes
  L = norm(E - S);  e = (E - S)/L;  nv = [-e(2), e(1)];
  pl = S + 0.50*N*e;                                    % piston plate
  cb = E - min(0.25*N, 0.9*L)*e;                        % cylinder bottom
  co = cb - 0.45*N*e;                                   % cylinder open end
  P = [S; pl; NaN NaN; pl + 0.8*w*nv; pl - 0.8*w*nv; NaN NaN; E; cb; NaN NaN; ...
       co + w*nv; cb + w*nv; cb - w*nv; co - w*nv];
end

function s = lag_string(td1, b)
% stride phase lag of the latest walker-2 leg-1 touchdown b against walker 1
  s = '';
  k = find(td1 <= b + 1e-9, 1, 'last');
  if isempty(k) || k < 2, return; end
  a = td1(k);  T = td1(k) - td1(k-1);
  lag = mod(360*(b - a)/T, 360);
  ref = [0 180 225 135 360];
  nm  = {'pace (~0)', 'trot (~180)', 'diagonal sequence (~225)', ...
         'diagonal sequence, mirrored (~135)', 'pace (~0)'};
  [dmin, m] = min(abs(lag - ref));
  if dmin <= 22.5
    s = sprintf('lag W2 vs W1: %3.0f deg  ->  near %s', lag, nm{m});
  else
    s = sprintf('lag W2 vs W1: %3.0f deg  ->  not near a Lanini reference', lag);
  end
end
