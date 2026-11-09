function m01_anim(block)
%
% Level-2 Matlab S-function for the animation of the muscle model
% The function has 5 input ports:
%
% Inputs:
% port 1: muscle force (port width: 1)
% port 2: length of contractile element (port width: 1)
% port 3: velocity of contractile element (port width: 1)
% port 4: length of series elasticity (port width: 1)
% port 5: muscle activation (port width: 1) 
%
% Dialog parameters passed into function:
% Parameter 1: animation realtime factor
% Parameter 2: model parameter structure
%
% hgeyer, Oct 2025
%

persistent TimingObj AnimObj MovieObj

MovieFlag = 1;  % set to 1 to enable movie recording       

setup(block);


% ##############################################################################
function setup(block)

  block.OperatingPointCompliance = 'Default';
  block.SampleTimes = [-1 0]; % inherits sample time  

  % register input ports and set their properties 
  block.NumInputPorts = 5;
  port_widths = [1, 1, 1, 1, 1];
  port_datatypes = [0, 0, 0, 0, 0];  % 0-double, 8-boolean
  for i = 1:block.NumInputPorts
    block.InputPort(i).Dimensions = port_widths(i);
    block.InputPort(i).DatatypeID = port_datatypes(i);
    block.InputPort(i).Complexity = 'Real';
    block.InputPort(i).DirectFeedthrough = true;
  end

  % register output ports and set their properties
  block.NumOutputPorts = 0;

  % set block dialog parameters
  block.NumDialogPrms = 2;  

  % register simulation functions
  block.RegBlockMethod('Start',         @Start);
  block.RegBlockMethod('Outputs',     @Outputs); 
  block.RegBlockMethod('Terminate', @Terminate);

end


% ##############################################################################
function Start(block)

  % set up animation figure and axes
  NameStr = 'Muscle Model';
  fig = findobj(0, 'type', 'figure', 'name', NameStr);
  if isempty(fig)
    AnimObj = [];
    AnimObj.fig = figure('Name', NameStr, 'NumberTitle', 'off', ...
                         'Color', [1 1 1], 'Position', [100 300 600 400]);
    AnimObj.ax1 = axes('FontSize', 8, 'Units', 'normalized', 'Color', 'w',  ...
                       'Position',  [0.07 0.56 0.25 0.41]);
    AnimObj.ax1.XLabel.String = 'l_{CE} [m]';
    AnimObj.ax1.YLabel.String = 'f_l [F_{max}]';
    
    AnimObj.ax2 = axes('FontSize', 8, 'Units', 'normalized', 'Color', 'w',  ...
                       'Position',  [0.40 0.56 0.25 0.41]);
    AnimObj.ax2.XLabel.String = 'v_{CE} [ms^{-1}]';
    AnimObj.ax2.YLabel.String = 'f_v [F_{max}]';
  
    AnimObj.ax3 = axes('FontSize', 8, 'Units', 'normalized', 'Color', 'w',  ...
                       'Position',  [0.73 0.56 0.25 0.41]);
    AnimObj.ax3.XLabel.String = 'l_{SE} [m]';
    AnimObj.ax3.YLabel.String = 'f_l [F_{max}]';
    
    AnimObj.ax4 = axes('FontSize', 8, 'Units', 'normalized', 'Color', 'w', ...
                      'Position',  [0.07 0.07 0.91 0.41]);
    AnimObj.ax4.XLabel.String = 'l_m [m]';
  else
    cla(AnimObj.ax1)  
    cla(AnimObj.ax2)  
    cla(AnimObj.ax3)  
    cla(AnimObj.ax4)  
  end
  
  mp = block.DialogPrm(2).Data;

  % assign muscle specific parameters
  lopt = mp.VAS.lopt;
  vmax = mp.VAS.vmax;
  lslack = mp.VAS.lslack;
  
  % CE AND PE FORCE LENGTH RELATIONS PLOT
  
  lceV = linspace(0, 2 * lopt, 50); 
  f_l_ceV = CE_force_length_relation(lceV / lopt, mp);  % active force length 
  f_l_peV = PE_force_length_relation(lceV / lopt, mp);
  
  AnimObj.ax1.XLim = [lceV(1) lceV(end)];
  AnimObj.ax1.YLim = [-0.01 1.02]; 
  hold(AnimObj.ax1, 'on')
  plot(AnimObj.ax1, [lopt lopt], [0 1], 'k:', 'LineWidth', 1)
  plot(AnimObj.ax1, lceV, f_l_ceV + f_l_peV, 'Color', [0.85 0.3 0.1], 'LineWidth', 2)
  plot(AnimObj.ax1, lceV, f_l_peV, 'Color', [0.9 0.7 0.1], 'LineWidth', 2)
  plot(AnimObj.ax1, lceV, f_l_ceV, 'Color', [0.1 0.5 0.8], 'LineWidth', 2)
  
  AnimObj.ax1fltotPnt = plot(AnimObj.ax1, lopt, 1, 'k.', 'MarkerSize', 25, 'Color', [0.85 0.3 0.1]);
  AnimObj.ax1flpePnt = plot(AnimObj.ax1, lopt, 0, 'k.', 'MarkerSize', 20, 'Color', [0.9 0.7 0.1]);
  AnimObj.ax1flcePnt = plot(AnimObj.ax1, lopt, 1, 'k.', 'MarkerSize', 20, 'Color', [0.1 0.5 0.8]);
  
  % CE FORCE VELOCITY RELATION
  
  vceV = linspace(-vmax * lopt, vmax * lopt, 50);
  f_vV = CE_force_velocity_relation(vceV / vmax / lopt, mp);
  
  AnimObj.ax2.XLim = [vceV(1) vceV(end)];
  AnimObj.ax2.YLim = [-0.01 mp.N + 0.02];
  hold(AnimObj.ax2, 'on')
  plot(AnimObj.ax2, [0 0], [0 mp.N], 'k:', 'LineWidth', 1)
  plot(AnimObj.ax2, vceV, f_vV, 'Color', [0.1 0.5 0.8], 'LineWidth', 2)
  
  AnimObj.ax2fvcePnt = plot(AnimObj.ax2, 0, 1, 'k.', 'MarkerSize', 20, 'Color', [0.1 0.5 0.8]);
  
  % SE FORCE LENGTH RELATION
  
  lseV = linspace(0, lslack * (1 + mp.eref * sqrt(mp.N)), 300);
  f_l_seV = SE_force_length_relation(lseV / lslack, mp);
  
  AnimObj.ax3.XLim = [lseV(1) lseV(end) * 1.01];
  AnimObj.ax3.YLim = [-0.01 mp.N + 0.02];
  hold(AnimObj.ax3, 'on')
  plot(AnimObj.ax3, [lslack lslack], [0 mp.N], 'k:', 'LineWidth', 1)
  plot(AnimObj.ax3, lseV, f_l_seV, 'Color', [0.1 0.5 0.8], 'LineWidth', 2)
  
  AnimObj.ax3flsePnt = plot(AnimObj.ax3, lslack, 0, 'k.', 'MarkerSize', 20, 'Color', [0.1 0.5 0.8]);
  
  % HILL-TYPE MODEL PLOT
  
  % generic spring and arc foot templates
  AnimObj.Spring.X = [0 2  3  4   5 6  7 8  9 10 12] / 12;
  AnimObj.Spring.Y = [0 0 -1  1  -1 1 -1 1 -1  0  0] * 0.2;
  
    
  AnimObj.ax4.XLim = [0 1.5 * mp.VAS.lopt + mp.VAS.lslack * (1 + mp.eref * sqrt(mp.N))]; 
  AnimObj.ax4.YLim = [-1 2];
  hold(AnimObj.ax4, 'on')
  AnimObj.ax4.YAxis.Visible = 'off';
  
  AnimObj.PE = plot(AnimObj.ax4, AnimObj.Spring.X * lopt, AnimObj.Spring.Y, ...
    'Color', [0.3 0.3 0.3], 'LineWidth', 3);
  AnimObj.CE1 = plot(AnimObj.ax4, [0 lopt lopt], [1 1 0], 'Color', [0.3 0.3 0.3], 'LineWidth', 3); 
  AnimObj.CE2 = plot(lopt / 2, 1, 'o', 'MarkerSize', 30, 'MarkerFaceColor', 'w', ...
    'MarkerEdgeColor', [0.3 0.3 0.3], 'LineWidth', 3);
  AnimObj.SE = plot(AnimObj.ax4, lopt + AnimObj.Spring.X * lslack, 0.5 + AnimObj.Spring.Y, ...
    'Color', [0.3 0.3 0.3], 'LineWidth', 3);
  plot(AnimObj.ax4, lopt, -1, '.', 'MarkerSize', 30, 'Color', [0.7 0.7 0.7]);
  plot(AnimObj.ax4, lopt + lslack, -1, '.', 'MarkerSize', 30, 'Color', [0.7 0.7 0.7]);
  
  AnimObj.ax4lceLn = plot(AnimObj.ax4, [lopt lopt], [-1 0.5], 'k.:', 'MarkerSize', 20);
  AnimObj.ax4lmLn = plot(AnimObj.ax4, lopt + lslack + [0 0], [-1 0.5], 'k.:', 'MarkerSize', 20);
  AnimObj.ax4fmArrow = line(AnimObj.ax4, lopt + lslack + [0 -0.5 * lopt], [0, 0], 'Color', [0.7 0.2 0.1], ...
    'LineWidth', 3, 'Marker', '<', 'MarkerSize', 10, 'MarkerIndices', 2, 'MarkerFaceColor', [0.7 0.2 0.1]);    

  drawnow  % ensures figure is present and up to date before proceeding

  % --- MOVIE SETUP ---
  
  if MovieFlag
    % adjust figure size to have even width and height required for mpeg-4
    if mod(fig.Position(3), 16) ~= 0
      fig.Position(3) = fig.Position(3) - mod(fig.Position(3), 16);
      drawnow
    end
    if mod(fig.Position(4), 16) ~= 0
      fig.Position(4) = fig.Position(4) - mod(fig.Position(4), 16);
      drawnow
    end

    % create mobie object
    MovieObj = VideoWriter('video.mp4', 'MPEG-4');
    MovieObj.Quality = 95;  %[0 ... 100]
    MovieObj.FrameRate = 60;
    open(MovieObj)
  end
   
  % --- ANIMATION TIMIMG SETUP ---

  TimingObj.LastWallTime = datetime;
  TimingObj.RefreshTime = 0;
  TimingObj.RealTimeFactor = block.DialogPrm(1).Data;
end
       

% ##############################################################################
function Outputs(block)
  
  % check if enough real-time has passed to update animation
  if block.CurrentTime >= TimingObj.RefreshTime
    
    % assign inputs
    fm = block.InputPort(1).Data;
    lce = block.InputPort(2).Data;
    vce = block.InputPort(3).Data;
    lse = block.InputPort(4).Data;
    act = block.InputPort(5).Data;
    mp = block.DialogPrm(2).Data;

    % assign muscle specific parameters
    lopt = mp.VAS.lopt;
    vmax = mp.VAS.vmax;
    lslack = mp.VAS.lslack;
    Fmax = mp.VAS.Fmax;
   
    % compute muscle force relations
    fl_ce = CE_force_length_relation(lce/lopt, mp);
    fl_pe = PE_force_length_relation(lce/lopt, mp);
    fv_ce = CE_force_velocity_relation(vce/vmax/lopt, mp);
    fl_se = SE_force_length_relation(lse/lslack, mp);

    % update animation objects
    AnimObj.ax1flcePnt.XData = lce;
    AnimObj.ax1flcePnt.YData = fl_ce;
    AnimObj.ax1flpePnt.XData = lce;
    AnimObj.ax1flpePnt.YData = fl_pe;
    AnimObj.ax1fltotPnt.XData = lce;
    AnimObj.ax1fltotPnt.YData = fl_ce + fl_pe;
    
    AnimObj.ax2fvcePnt.XData = vce;
    AnimObj.ax2fvcePnt.YData = fv_ce;
    
    AnimObj.ax3flsePnt.XData = lse;
    AnimObj.ax3flsePnt.YData = fl_se;
  
    AnimObj.ax4lceLn.XData = [lce lce];
    AnimObj.ax4lmLn.XData = lce + lse + [0 0];
    AnimObj.PE.XData = AnimObj.Spring.X * lce;
    AnimObj.PE.LineStyle = char(45 + 13 * (lce < lopt));  
    AnimObj.CE1.XData = [0 lce lce];
    AnimObj.CE2.XData = 0.5 * lce;
    AnimObj.CE2.MarkerFaceColor = [1, 1 - act, 1 - act];
    AnimObj.SE.XData = lce + AnimObj.Spring.X * lse;
    AnimObj.SE.LineStyle = char(45 + 13 * (lse < lslack));  
    AnimObj.ax4fmArrow.XData = lce + lse + [0 -fm/Fmax * lopt];
    
    drawnow nocallbacks
    
    if block.CurrentTime == 0, shg, end
    
    % add current frame to video
    if MovieFlag
      writeVideo(MovieObj, getframe(AnimObj.fig)); 
    end

    % update refresh time
    if MovieFlag
      TimingObj.RefreshTime = block.CurrentTime + 1 * 1 / MovieObj.FrameRate;
    else
      CurrWallTime = datetime;
      TimeDiff = seconds(CurrWallTime - TimingObj.LastWallTime);
      TimeStep = TimingObj.RealTimeFactor * TimeDiff;
      TimingObj.LastWallTime = CurrWallTime;
      % TimeStep = 0.001;
      TimingObj.RefreshTime = block.CurrentTime + TimeStep;
    end 
 
  end
end


% ##############################################################################
function Terminate(block)
  if MovieFlag && ~isempty(MovieObj)
    close(MovieObj); 
  end
end

end