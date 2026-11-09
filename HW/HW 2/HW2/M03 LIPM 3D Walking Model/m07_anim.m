function m07_anim(block)
%
% Level-2 Matlab S-function for the animation of 3-D LIPM Gait. 
% The function has 7 input ports:
%
% Inputs:
% port 1: CoM xyz position (port width: 3)
% port 2: Right leg foot point xyz position (port width: 3)
% port 3: Left leg foot point xyz position (port width: 3)
% port 4: Right leg state (1: stance, 0: swing) (port width: 1) 
% port 5: Left leg state (1: stance, 0: swing) (port width: 1) 
% port 6: Right leg nominal foot point xyz position (port width: 3) 
% port 7: Leg leg nominal foot point xyz position (port width: 3) 
%
% Diaolog parameters passed into the function:
% Parameter 1: model parameters
% Parameter 2: animation update time step
% Parameter 3: ground profile x-points
% Parameter 4: ground profile y-points
% Parameter 5: ground profile z-elevation
%
% H.Geyer, Sep 2025
%

persistent hFig TimingObj AnimObj MovieObj

MovieFlag = 0;         

% view window
ViewXRange = 6;  % [m]  
ViewYRange = 1;  % [m]
ReRange   = 0.0; % [m]       

setup(block);


% ##############################################################################
function setup(block)

  % specify the block simStateCompliance
  block.SimStateCompliance = 'DefaultSimState';

  % --- Register Block I/O Ports ----------------------------------------------- 

  block.NumInputPorts  = 7;
  block.NumOutputPorts = 0;

  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;

  % input 1: CoM position
  block.InputPort(1).Dimensions  = 3;
  block.InputPort(1).DatatypeID  = 0;  
  block.InputPort(1).Complexity  = 'Real';
  block.InputPort(1).DirectFeedthrough = true;

  % input 2: Right Foot Point position
  block.InputPort(2).Dimensions  = 3;
  block.InputPort(2).DatatypeID  = 0;  
  block.InputPort(2).Complexity  = 'Real';
  block.InputPort(2).DirectFeedthrough = true;

  % input 3: Left Foot Point position
  block.InputPort(3).Dimensions  = 3;
  block.InputPort(3).DatatypeID  = 0;  
  block.InputPort(3).Complexity  = 'Real';
  block.InputPort(3).DirectFeedthrough = true;

  % input 4: Right stance/swing state
  block.InputPort(4).Dimensions  = 1;
  block.InputPort(4).DatatypeID  = 8;  % boolean
  block.InputPort(4).DirectFeedthrough = true;

  % input 5: Left stance/swing state
  block.InputPort(5).Dimensions  = 1;
  block.InputPort(5).DatatypeID  = 8;  % boolean
  block.InputPort(5).DirectFeedthrough = true;
  
  % input 6: Nominal right foot point
  block.InputPort(6).Dimensions  = 3;
  block.InputPort(6).DatatypeID  = 0;  
  block.InputPort(6).Complexity  = 'Real';
  block.InputPort(6).DirectFeedthrough = true;

  % input 7: Nominal left foot point target
  block.InputPort(7).Dimensions  = 3;
  block.InputPort(7).DatatypeID  = 0;  
  block.InputPort(7).Complexity  = 'Real';
  block.InputPort(7).DirectFeedthrough = true;


  % --- Register Parameters ----------------------------------------------------
  
  block.NumDialogPrms = 5;  % model parametre structure, real time factor, 
                            % ground x-points, y-points, and z-elevations

  % --- Set Block Sample Time -------------------------------------------------- 

  block.SampleTimes = [-1 0];  % sets to inherit sample time  

  % --- Register Simulation Functions ------------------------------------------

  block.RegBlockMethod('Start',         @Start);
  block.RegBlockMethod('PostPropagationSetup', @PostPropagationSetup);
  block.RegBlockMethod('Outputs',     @Outputs); % required
  block.RegBlockMethod('Update',       @Update);
  block.RegBlockMethod('Terminate', @Terminate); % required

end

% ##############################################################################
function PostPropagationSetup(block)
    
  % Register and Init Memory States --------------------------------------------
  block.NumDworks = 1;
  block.Dwork(1).Name = 'leg_phase';
  block.Dwork(1).Dimension = 2;
  block.Dwork(1).DatatypeID = 8;  % boolean
  block.Dwork(1).Complexity = 'Real';
  block.Dwork(1).UsedAsDiscState = true;
end  

% ##############################################################################
function Start(block)
  
  % --- Initialize Animation Figure --------------------------------------------

  NameStr = '3-D Linear Inverted Pendulum Gait Model';

  hFig = findobj(0, 'type', 'figure', 'name', NameStr);
  if isempty(hFig)
    hFig = figure('Name', NameStr, 'NumberTitle', 'off', ...
		              'Color',  [1 1 1], 'Position', [100   100   800   400]);
    axes('Parent', hFig, 'FontSize', 8, 'Units', 'normalized', ...
         'Position',  [0.01 0.01 0.98 0.98]);
  end

  cla reset
  ax = gca;
  set(ax, 'SortMethod', 'childorder', 'Visible', 'on', ...
      'Color', [1 1 1], 'XColor',  [0 0 0], 'YColor', [0 0 0]);
  hold on
  grid on

  axis([-0.5, -0.5 + ViewXRange, - ViewYRange / 2, ViewYRange / 2, -0.01, 0.89]);
  
  view(-40, 30)
  daspect([1 1 1])
  xlabel('fore-aft (x) [m]')
  ylabel('medio-lateral (y) [m]')
  zlabel('height [m]')  

  
  % --- Create Animation Objects -----------------------------------------------

  mp = block.DialogPrm(1).Data;

  AnimObj = [];

  AnimObj.Gnd = surf(block.DialogPrm(3).Data, block.DialogPrm(4).Data, ...
                     block.DialogPrm(5).Data, ...
                     'FaceColor', [0.8 0.8 0.8], 'EdgeColor', [0.6 0.6 0.6]); 

  [Xcyl, Ycyl, Zcyl] = cylinder(1, 10);   % generic cylinder 
  [Xsph, Ysph, Zsph] = sphere(20);        % and sphere 

  % center body  
  AnimObj.CenterBody = hgtransform('Visible', 'on', 'HitTest', 'off', 'PickableParts', 'none');
  scl = 0.1;
  surface(Xsph * scl, Ysph * scl, Zsph * scl, ...
          'Parent', AnimObj.CenterBody, 'EdgeColor', 'none', 'FaceColor', [0 1 0]);

  % right leg
  AnimObj.RLegCyl = hgtransform('Visible', 'on', 'HitTest', 'off', 'PickableParts', 'none', ...
                                'Parent', AnimObj.CenterBody);
  surface(Xcyl * 0.03, Ycyl * 0.03, 0.6 * mp.z0 * Zcyl - 0.7 * mp.z0, ... 
          'Parent', AnimObj.RLegCyl, 'EdgeColor','none', 'FaceColor', [1 0 0]);

  AnimObj.RLegPist = hgtransform('Visible', 'on', 'HitTest', 'off', 'PickableParts', 'none', ...
                                 'Parent', AnimObj.RLegCyl);
  surface(Xcyl * 0.02, Ycyl * 0.02, 0.9 * mp.z0 * Zcyl - mp.z0, ... 
          'Parent', AnimObj.RLegPist, 'EdgeColor','none', 'FaceColor', [1 0 0]);
  surface(Xsph * 0.02, Ysph * 0.02, Zsph * 0.02 - mp.z0, ...
          'Parent', AnimObj.RLegPist, 'EdgeColor', 'none', 'FaceColor', [1 0.5 0.5]);

  % left leg
  AnimObj.LLegCyl = hgtransform('Visible', 'on', 'HitTest', 'off', 'PickableParts', 'none', ...
                                'Parent', AnimObj.CenterBody);
  surface(Xcyl * 0.03, Ycyl * 0.03, 0.6 * mp.z0 * Zcyl - 0.7 * mp.z0, ... 
          'Parent', AnimObj.LLegCyl, 'EdgeColor','none', 'FaceColor', [0 0 1]);

  AnimObj.LLegPist = hgtransform('Visible', 'on', 'HitTest', 'off', 'PickableParts', 'none', ...
                                 'Parent', AnimObj.LLegCyl);
  surface(Xcyl * 0.02, Ycyl * 0.02, 0.9 * mp.z0 * Zcyl - mp.z0, ... 
          'Parent', AnimObj.LLegPist, 'EdgeColor','none', 'FaceColor', [0 0 1]);
  surface(Xsph * 0.02, Ysph * 0.02, Zsph * 0.02 - mp.z0, ...
          'Parent', AnimObj.LLegPist, 'EdgeColor', 'none', 'FaceColor', [0.5 0.5 1]);
  
  camlight right
  lighting flat
  material shiny  
  light

  % initialize leg states memory
  block.Dwork(1).Data = [false true];

 
  % --- Init Movie Recording ---------------------------------------------------
  if MovieFlag
    % adjust figure size to have even width and height required for mpeg-4
    if mod(hFig.Position(3), 2) ~= 0
        hFig.Position(3) = hFig.Position(3) + 1;
    end
    if mod(hFig.Position(4), 2) ~= 0
        hFig.Position(4) = hFig.Position(4) + 1;
    end
    
    % create movie object
    MovieObj = VideoWriter('video.mp4', 'MPEG-4');
    MovieObj.Quality = 95;  %[0 ... 100]
    MovieObj.FrameRate = 50;
    open(MovieObj)
  end
   
  % --- Create and Init Timing Object ------------------------------------------
  
  TimingObj = [];
  TimingObj.LastWallTime = datetime;
  TimingObj.RefreshTime = 0;
  TimingObj.RealTimeFactor = block.DialogPrm(2).Data;

end
       

% ##############################################################################
function Outputs(block)
  % nothing to do here
end


% ##############################################################################
function Update(block)
 
  % update animation figure
  if block.CurrentTime >= TimingObj.RefreshTime
    
    set(0, 'currentfigure', hFig)
    
    % assign inputs
    CM = block.InputPort(1).Data;
    RFP = block.InputPort(2).Data;
    LFP = block.InputPort(3).Data;

    Rstate = block.InputPort(4).Data;
    Rprevstate = block.Dwork(1).Data(1);
    Lstate = block.InputPort(5).Data;
    Lprevstate = block.Dwork(1).Data(2);
    
    RFPtgt = block.InputPort(6).Data;
    LFPtgt = block.InputPort(7).Data;

    mp = block.DialogPrm(1).Data;
 
    % adjust axes if CoM leaves view range 
    ax = gca;
    if ax.XLim(2) < (CM(1) + ReRange)
      ax.XLim = CM(1) - ReRange + [0, ViewXRange];
    end
    if ax.XLim(1) > (CM(1) - ReRange)
      ax.XLim = CM(1) - ReRange + [0, ViewXRange];
    end
    if ax.YLim(2) < (CM(2) + ReRange)
      ax.YLim = CM(2) + [-0.5 * ViewYRange, 0.5 * ViewYRange];
    end
    if ax.YLim(1) > (CM(2) - ReRange)
      ax.YLim = CM(2) + [-0.5 * ViewYRange, 0.5 * ViewYRange];
    end

    % update center body 
    AnimObj.CenterBody.Matrix = [eye(3), CM; 0 0 0 1];
    plot3(CM(1), CM(2), CM(3), 'k.', 'MarkerSize', 3)
    
    % update right leg
    rvec = CM - RFP;
    pitch = atan2(sqrt(rvec(1)^2 + rvec(2)^2), rvec(3));
    cp = cos(pitch); sp = sin(pitch);
    Ry = [cp 0 sp; 0 1 0; -sp 0 cp];
    
    yaw = atan2(rvec(2), rvec(1));
    cy = cos(yaw); sy = sin(yaw);
    Rz = [cy -sy 0; sy cy 0; 0 0 1]; 
    
    AnimObj.RLegCyl.Matrix = [Rz * Ry, [0; 0; 0]; 0, 0, 0, 1];

    dist = mp.z0 - norm(rvec);
    AnimObj.RLegPist.Matrix = [eye(3), [0; 0; dist]; 0, 0, 0, 1];
    
    if (Rstate == 1) && (Rprevstate == 0) 
        AnimObj.RLegCyl.Children(2).FaceAlpha = 1;
        AnimObj.RLegPist.Children(2).FaceAlpha = 1;
        plot3(RFP(1), RFP(2), RFP(3), 'r.', 'MarkerSize', 10)
        plot3(LFPtgt(1), LFPtgt(2), LFPtgt(3), 'bo', 'MarkerSize', 5)
    end

    if (Rstate == 0) && (Rprevstate == 1) 
        AnimObj.RLegCyl.Children(2).FaceAlpha = 0.1;
        AnimObj.RLegPist.Children(2).FaceAlpha = 0.1;
    end

    % update left leg
    lvec = CM - LFP;
    pitch = atan2(sqrt(lvec(1)^2 + lvec(2)^2), lvec(3));
    cp = cos(pitch); sp = sin(pitch);
    Ry = [cp 0 sp; 0 1 0; -sp 0 cp];
    
    yaw = atan2(lvec(2), lvec(1));
    cy = cos(yaw); sy = sin(yaw);
    Rz = [cy -sy 0; sy cy 0; 0 0 1]; 
    
    AnimObj.LLegCyl.Matrix = [Rz * Ry, [0; 0; 0]; 0, 0, 0, 1];
   
    dist = mp.z0 - norm(lvec);
    AnimObj.LLegPist.Matrix = [eye(3), [0; 0; dist]; 0, 0, 0, 1];
    
    if (Lstate == 1) && (Lprevstate == 0) 
        AnimObj.LLegCyl.Children(2).FaceAlpha = 1;
        AnimObj.LLegPist.Children(2).FaceAlpha = 1;
        plot3(LFP(1), LFP(2), LFP(3), 'b.', 'MarkerSize', 10)
        plot3(RFPtgt(1), RFPtgt(2), RFPtgt(3), 'ro', 'MarkerSize', 5)
    end

    if (Lstate == 0) && (Lprevstate == 1) 
        AnimObj.LLegCyl.Children(2).FaceAlpha = 0.1;
        AnimObj.LLegPist.Children(2).FaceAlpha = 0.1;
    end

    % save current leg states
    block.Dwork(1).Data = [Rstate Lstate];


    drawnow nocallbacks
    
    if block.CurrentTime == 0, shg, end

    % add current frame to video
    if MovieFlag
      writeVideo(MovieObj, getframe(gcf)); 
    end

    % update refresh time
    if MovieFlag
        TimingObj.RefreshTime = block.CurrentTime + 1 / MovieObj.FrameRate;
    else
        CurrWallTime = datetime;
        TimeDiff = seconds(CurrWallTime - TimingObj.LastWallTime);
        TimeStep = TimingObj.RealTimeFactor * TimeDiff;
        TimingObj.LastWallTime = CurrWallTime;
      
        TimingObj.RefreshTime = block.CurrentTime + TimeStep;
    end 
  end
end

% ##############################################################################
function Terminate(block)
  if MovieFlag
    close(MovieObj); 
  end
end

end