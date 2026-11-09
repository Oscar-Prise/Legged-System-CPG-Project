% 
% NMS_Model_MechInit.m  - Set mechanical parameters of the model.
%
%    This parameter setup includes: 
%    1. segment geometries, masses and inertias,
%    2. muscle-skeleton mechanical links,
%    3. muscle mechanics, and
%    4. ground interaction.
%
% H.Geyer
% 5 October 2006
%


% gravity
g = 9.81;



% ********************* %
% 1. BIPED SEGMENTATION %
% ********************* %

% ------------------------
% 1.1 General Foot Segment
% ------------------------

% Note: CS1 is position of fore-foot impact point and contacts 
%       the adjoining ground.

% foot length
L_F = 0.2; %[m]

% distance CS1 to CG (local center of gravity)
D1G_F = 0.14; %[m]0.13 % critical to project

% distance CS1 to C2 (ankle joint)
D12_F = 0.16; %[m]16

% distance CS1 to CS3 (heel pad impact point)
D13_F = 0.20; %[m]20

% foot mass
m_F  = 1.25; %[kg] 1.25

% foot inertia with respect to axis ball-CG-heel (scaled from other papers)
In_F = 0.005; %[kg*m^2]



% -------------------------
% 1.2 General Shank Segment
% -------------------------

% Note: CS1 is position of ankle joint.

% shank length
L_S = 0.5; %[m]

% distance CS1 to CG (local center of gravity)
D1G_S = 0.3; %[m]

% distance CS1 to C2 (knee joint)
D12_S = L_S; %[m]

% shank mass
m_S  = 3.5; %[kg]

% shank inertia with respect to axis ankle-CG-knee (scaled from other papers)
In_S = 0.05; %[kg*m^2]



% -------------------------
% 1.3 General Thigh Segment
% -------------------------

% Note: CS1 is position of knee joint.

% total thigh length
L_T = 0.5; %[m]

% distance CS1 to CG (local center of gravity)
D1G_T = 0.3; %[m] 

% distance CS1 to C2 (hip joint)
D12_T = L_T; %[m]

% thigh mass
m_T  = 8.5; %[kg]

% thigh inertia with respect to axis ball-CG-heel (scaled from other papers)
In_T = 0.15; %[kg*m^2]



% -----------------------------------------
% 1.4 General Head-Arms-Trunk (HAT) Segment
% -----------------------------------------

% HAT length
L_HAT = 0.8; %[m]

% distance hip to center of gravity
D1G_HAT = 0.35; %[m]

% HAT mass
m_HAT = 53.5; %[kg]

% HAT inertia (scaled from other papers)
In_HAT = 3; %[kg*m^2] 



% --------------------------------
% 1.5 Thigh Segment Pressure Sheet
% --------------------------------

% reference compression corresponding to steady-state with HAT mass
DeltaThRef = 5e-3; %[m]

% interaction stiffness
k_pressure = m_HAT * 9.81 / DeltaThRef; %[N/m]

% max relaxation speed (one side)
v_max_pressure = 0.5; %[m/s]

% position in thigh
ThPos = 4/5; % in length of thigh, measured from lower thigh end



% ---------------------
% 1.6 Joint Soft Limits
% ---------------------

% angles at which soft limits engages
phi12_LowLimit =  70*pi/180; %[rad]
phi12_UpLimit  = 130*pi/180; %[rad]

phi23_UpLimit  = 175*pi/180; %[rad]

phi34_UpLimit  = 230*pi/180; %[rad]

% soft block reference joint stiffness
c_jointstop     = 0.3 / (pi/180);  %[Nm/rad]

% soft block maximum joint stop relaxation speed
w_max_jointstop = 1 * pi/180; %[rad/s]





% ****************************** %
% 2. BIPED MUSCLE-SKELETON-LINKS %
% ****************************** %

% ----------------------------------------
% 2.1 Joint Specific Link Parameters
% ----------------------------------------

% SOLeus attachement
rSOL       =       0.05; % [m] radius 
phimaxSOL  = 110*pi/180; % [rad] angle of maximum lever contribution
phirefSOL  =  80*pi/180; % [rad] reference angle at which MTU length equals 
rhoSOL     =        0.5; %       sum of lopt and lslack 

% Tibialis Anterior attachement
rTA       =        0.04; % [m]   constant lever contribution 
phimaxTA   =  80*pi/180; % [rad] angle of maximum lever contribution
phirefTA   = 110*pi/180; % 100[rad] reference angle at which MTU length equals 
rhoTA      =        0.7; 

% GAStrocnemius attachement (ankle joint)
rGASa      =       0.05; % [m]   constant lever contribution 
phimaxGASa = 110*pi/180; % [rad] angle of maximum lever contribution
phirefGASa =  80*pi/180; % [rad] reference angle at which MTU length equals 
rhoGASa    =        0.7; %       sum of lopt and lslack 

% GAStrocnemius attachement (knee joint)
rGASk      =       0.05; % [m]   constant lever contribution 
phimaxGASk = 140*pi/180; % [rad] angle of maximum lever contribution
phirefGASk = 165*pi/180; % [rad] reference angle at which MTU length equals 
rhoGASk    =        0.7; %       sum of lopt and lslack 

% VAStus group attachement
rVAS       =       0.06; % [m]   constant lever contribution 
phimaxVAS  = 165*pi/180; % [rad] angle of maximum lever contribution
phirefVAS  = 125*pi/180; % [rad] reference angle at which MTU length equals 
rhoVAS     =        0.7; %       sum of lopt and lslack 
                       
% HAMstring group attachement (knee)
rHAMk      =       0.05; % [m]   constant lever contribution 
phimaxHAMk = 180*pi/180; % [rad] angle of maximum lever contribution
phirefHAMk = 180*pi/180; % [rad] reference angle at which MTU length equals 
rhoHAMk    =        0.7; %       sum of lopt and lslack 
                         
% HAMstring group attachement (knee)
rHAMh      =       0.08; % [m]   constant lever contribution 
phirefHAMh = 155*pi/180; % [rad] reference angle at which MTU length equals 
rhoHAMh    =        0.7; %       sum of lopt and lslack 

% GLUtei group attachement
rGLU       =       0.10; % [m]   constant lever contribution 
phirefGLU  = 150*pi/180; % [rad] reference angle at which MTU length equals 
rhoGLU     =        0.5; %       sum of lopt and lslack 
                         
% Hip FLexor group attachement
rHFL       =       0.10; % [m]   constant lever contribution 
phirefHFL  = 180*pi/180; % [rad] reference angle at which MTU length equals 
rhoHFL     =        0.5; %       sum of lopt and lslack 


                         

% ************************* %
% 3. BIPED MUSCLE MECHANICS %
% ************************* %

% -----------------------------------
% 3.1 Shared Muscle Tendon Parameters
% -----------------------------------

% excitation-contraction coupling
preA =  0.01; %[] preactivation
tau  =  0.01; %[s] delay time constant

% contractile element (CE) force-length relationship
w    =   0.56; %[lopt] width
c    =   0.05; %[]; remaining force at +/- width

% CE force-velocity relationship
N    =   1.5; %[Fmax] eccentric force enhancement
K    =     5; %[] shape factor

% Series elastic element (SE) force-length relationship
eref =  0.04; %[lslack] tendon reference strain



% ------------------------------
% 3.2 Muscle-Specific Parameters
% ------------------------------

% soleus muscle
FmaxSOL    = 4000; % maximum isometric force [N]
loptSOL    = 0.04; % optimum fiber length CE [m]
vmaxSOL    =    6; % maximum contraction velocity [lopt/s]
lslackSOL  = 0.26; % tendon slack length [m]

% gastrocnemius muscle
FmaxGAS    = 1500; % maximum isometric force [N]
loptGAS    = 0.05; % optimum fiber length CE [m]
vmaxGAS    =   12; % maximum contraction velocity [lopt/s]
lslackGAS  = 0.40; % tendon slack length [m]

% tibialis anterior
FmaxTA     =  800; % maximum isometric force [N]
loptTA     = 0.06; % optimum fiber length CE [m]
vmaxTA     =   12; % maximum contraction velocity [lopt/s]
lslackTA   = 0.24; % tendon slack length [m]

% vasti muscles
FmaxVAS    = 6000; % maximum isometric force [N]
loptVAS    = 0.08; % optimum fiber length CE [m]
vmaxVAS    =   12; % maximum contraction velocity [lopt/s]
lslackVAS  = 0.23; % tendon slack length [m]

% hamstring muscles
FmaxHAM   = 3000; % maximum isometric force [N]
loptHAM   = 0.10; % optimum fiber length CE [m]
vmaxHAM   =   12; % maximum contraction velocity [lopt/s]
lslackHAM = 0.31; % tendon slack length [m]

% glutei muscles
FmaxGLU   = 1500; % maximum isometric force [N]
loptGLU   = 0.11; % optimum fiber length CE [m]
vmaxGLU   =   12; % maximum contraction velocity [lopt/s]
lslackGLU = 0.13; % tendon slack length [m]

% glutei muscles
FmaxHFL   = 2000; % maximum isometric force [N]
loptHFL   = 0.11; % optimum fiber length CE [m]
vmaxHFL   =   12; % maximum contraction velocity [lopt/s]
lslackHFL = 0.10; % tendon slack length [m]



% --------------------------------
% 3.3 Vectorized Muscle-Parameters
% --------------------------------

% sorted by:
% left: SOL TA GAS VAS HAM GLU HFL and then
% right: SOL TA GAS VAS HAM GLU HFL

% Left/Right Muscle Separate
if 0
  
  % optimum muscle lengths
  loptVec = [loptSOL loptTA loptGAS loptVAS loptHAM loptGLU loptHFL];% ...

  % tendon slack lengths         
  lslackVec = [lslackSOL lslackTA lslackGAS lslackVAS lslackHAM lslackGLU lslackHFL];% ...

  % maximum contraction velocities     
  vmaxVec = [vmaxSOL vmaxTA vmaxGAS vmaxVAS vmaxHAM vmaxGLU vmaxHFL];%...

  % maximum isometric forces         
  FmaxVec = [FmaxSOL FmaxTA FmaxGAS FmaxVAS FmaxHAM FmaxGLU FmaxHFL];% ...

% Left/Right Muscles Combined  
else

  % optimum muscle lengths
  loptVec = [loptSOL loptTA loptGAS loptVAS loptHAM loptGLU loptHFL ...
             loptSOL loptTA loptGAS loptVAS loptHAM loptGLU loptHFL]';

  % tendon slack lengths         
  lslackVec = [lslackSOL lslackTA lslackGAS lslackVAS lslackHAM lslackGLU lslackHFL ...
               lslackSOL lslackTA lslackGAS lslackVAS lslackHAM lslackGLU lslackHFL]';

  % maximum contraction velocities     
  vmaxVec = [vmaxSOL vmaxTA vmaxGAS vmaxVAS vmaxHAM vmaxGLU vmaxHFL ...
             vmaxSOL vmaxTA vmaxGAS vmaxVAS vmaxHAM vmaxGLU vmaxHFL]'; 

  % maximum isometric forces         
  FmaxVec = [FmaxSOL FmaxTA FmaxGAS FmaxVAS FmaxHAM FmaxGLU FmaxHFL ...
             FmaxSOL FmaxTA FmaxGAS FmaxVAS FmaxHAM FmaxGLU FmaxHFL]';
         
end

         
% *************************** %
% 4. Ground Interaction Model %
% *************************** %

% ----------------------
% 4.1 Vertical component
% ----------------------

% stiffness of vertical ground interaction
k_gy = (2*(m_F+m_S+m_T)+m_HAT)*g/0.01; %[N/m]

% max relaxation speed of vertical ground interaction
v_gy_max = 0.03; %[m/s]



% ------------------------
% 4.2 Horizontal component
% ------------------------

% sliding friction coefficient
mu_slide = 0.8;

% sliding to stiction transition velocity limit
vLimit = 0.01; %[m/s]

% stiffness of horizontal ground stiction
k_gx = (2*(m_F+m_S+m_T)+m_HAT)*g/0.1; %[N/m] 0.01

% max relaxation speed of horizontal ground stiction
v_gx_max = 0.03; %[m/s] 0.03

% stiction to sliding transition coefficient
mu_stick = 0.9;














