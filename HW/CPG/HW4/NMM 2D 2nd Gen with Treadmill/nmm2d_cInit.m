
% 
% nms_model_ControlInit.m  -  Set neural control parameters of the model
%                             as well as initial conditions and simulink 
%                             control parameters.
%
% H.Geyer
% 5 October 2006
%





% ************************************ %
% 1. General Neural Control Parameters %
% ************************************ %

% feedback delays
LongDelay  = 0.020; % ankle joint muscles [s]
MidDelay   = 0.010; % knee joint muscles [s]
ShortDelay = 0.005; % hip joint muscles [s]





% ****************************** %
% 2. Specific Control Parameters %
% ****************************** %



% -------------------------------
% 2.1 Stance-Leg Feedback Control 
% -------------------------------

% soleus (self, F+)
GainSOL      =   1.2/FmaxSOL; %[1/N]
PreStimSOL   =         0.01; %[]

% soleus on tibialis anterior (F-)
GainSOLTA    =     0.4/FmaxSOL; %[1/N]
PreStimTA    =          0.01; %[]

% tibialis (self, L+, stance & swing)
GainTA       =         1.1;   %[]
LceOffsetTA  =       1-0.5*w; %[loptTA]

% gastrocnemius (self, F+)
GainGAS      =   1.1/FmaxGAS; %[1/N] 
PreStimGAS   =         0.01; %[]

% vasti group (self, F+)
GainVAS      =    1.2/FmaxVAS; %[1/N]
PreStimVAS   =         0.08; %[]


% knee overextension on vasti (Phi-, directional)
GainKneOverExt = 2;%
KneePh23Offset = 170*pi/180;

% swing initiation
K_swing = 0.25;%0.25



% ------------------------------
% 2.1 Swing-leg Feedback Control 
% ------------------------------


% Fly
% ---

% hip flexors (self, L+, swing)
LceOffsetHFL =        0.65; %[loptHFL]
PreStimHFL   =        0.01; %[] 
GainHFL      =         0.5; %[] 

% balance offset shift (Delta Theta at take-off)
GainDeltaTheta = 2 /100*180/pi; %[percent/deg]


% Catch
% -----

% hip flexor from hamstring stretch reflex (L-, swing)
LceOffsetHAM =        0.85; %[loptHAM]
GainHAMHFL   =           4; %[]

% hamstring group (self, F+, swing)
GainHAM      =  0.65/FmaxHAM; %[1/N]
PreStimHAM   =         0.01; %[]

% gluteus group (self, F+, swing)
GainGLU      =  0.5/FmaxGLU; %[1/N]
PreStimGLU   =         0.01; %[]



% -----------------------------------------------
% 2.2 Stance-Leg HAT Reference Posture PD-Control
% -----------------------------------------------

% stance hip joint position gain
PosGainGG   = 1/(30*pi/180); %[1/rad]

% stance hip joint speed gain
SpeedGainGG = 0.2; %[s/rad] 

% stance posture control muscles pre-stimulation
PreStimGG   = 0.05; %[]

% stance reference posture
Theta0      = 6*pi/180; %[rad]





% ******************************************** %
% 3. Initial Conditions and Simulation Control %
% ******************************************** %



% ----------------------
% 3.1 Initial Conditions
% ----------------------

% initial locomotion speed
vx0 = 1.3; %[m/s] 

% left (stance) leg ankle, knee and hip joint angles
Lphi120  =  85*pi/180; %[rad]
Lphi230  = 175*pi/180; %[rad]
Lphi340  = 175*pi/180; %[rad]

% right (swing) leg ankle, knee and hip joint angles
Rphi120  =  90*pi/180; %[rad]
Rphi230  = 175*pi/180; %[rad]
Rphi340  = 140*pi/180; %[rad]

% compute floating base initial position from initial angles assuming the
% left leg ball position is at [0 0]
Rot120 = [cos( Lphi120) -sin( Lphi120); sin( Lphi120) cos( Lphi120)];
Rot230 = [cos(-Lphi230) -sin(-Lphi230); sin(-Lphi230) cos(-Lphi230)];
Rot340 = [cos( Lphi340) -sin( Lphi340); sin( Lphi340) cos( Lphi340)];

HAT_CG0 = [-D12_F; 0] + Rot120*[L_S;0] + Rot120*Rot230*[-L_T;0] ...
          + Rot120*Rot230*Rot340*[D1G_HAT;0]  


% ----------------------
% 3.2 Simulation Control
% ----------------------


% Animations
% ----------

% integrator max time step
ts_max = 1e-1;




