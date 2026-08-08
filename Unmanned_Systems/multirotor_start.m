clear
addpath('cad_parts')

%% user specifications
controller_mode = 1;  % 0: attitude, 1: velocity
set_point_input = 0;  % 0: camera (interception), 1: joy stick, 2: from script

%% simulation solver parameter
% Runge-Kutta 4th-order solver
ST = 0.01;  %[s]

%% initial conditions
Xe_0 = [0 0 -30]';  %[m] initial position of CG of MR in NED frame (North East Down)
V0 = [0 0 0]';  %[m/s] initial velocity of MR in body-frame
euler0 = [0 0 0]';  %[rad] initial attitude of MR (roll, pitch, yaw)
omega0 = [0 0 0]';  %[rad/s] initial angular velocity of MR in body frame

latref = 44.200626;  %[deg] reference latitude for earth fixed origin
lonref = 12.063822;  %[deg] reference longitude for earth fixed origin
href = 0;  %[m] reference altitude for earth fixed origin (0 = surface, !=sea lvl)

%% MR inertial parameters
m = 2.15; %[kg]  MR take-off mass
J = diag([0.0319 0.0287 0.0633]);  %[kg m^2] MR inertia matrix in body frame, about CG
STACG = 0;  %[m] stationline of CG
BLCG = 0;  %[m] buttline of CG
WLCG = -0.03;  %[m] waterline of CG

%% environmental conditions
wind_velocity_e = [0 0 0]';  %[m/s] wind speed in NED frame
wind_rate = [0 0 0]';  %[rad/s] wind rate in body-frame
dyn_visco = 17.89e-6;  %[Pa s] reference air dynamic viscosity

%% MR configuration parameters
b = 0.55/2;  %[m] MR arm lenght

STAH1 = -b*cosd(30);  %[m] stationline of rot1
BLH1 = b*sind(30);  %[m] buttline of rot1
WLH1 = 32e-3;  %[m] waterline of rot1
chi1 = 1;  %[] 1 for counterclockwise; -1 for clockwise

STAH2 = 0;  %[m] stationline of rot2
BLH2 = b;  %[m] buttline of rot2
WLH2 = 32e-3;  %[m] waterline of rot2
chi2 = -1;  %[] 1 for counterclockwise; -1 for clockwise

STAH3 = b*cosd(30);  %[m] stationline of rot3
BLH3 = b*sind(30);  %[m] buttline of rot3
WLH3 = 32e-3;  %[m] waterline of rot3
chi3 = 1;  %[] 1 for counterclockwise; -1 for clockwise

STAH4 = b*cosd(30);  %[m] stationline of rot4
BLH4 = -b*sind(30);  %[m] buttline of rot4
WLH4 = 32e-3;  %[m] waterline of rot4
chi4 = -1;  %[] 1 for counterclockwise; -1 for clockwise

STAH5 = 0;  %[m] stationline of rot5
BLH5 = -b;  %[m] buttline of rot5
WLH5 = 32e-3;  %[m] waterline of rot5
chi5 = 1;  %[] 1 for counterclockwise; -1 for clockwise

STAH6 = -b*cosd(30);  %[m] stationline of rot6
BLH6 = -b*sind(30);  %[m] buttline of rot6
WLH6 = 32e-3;  %[m] waterline of rot6
chi6 = -1;  %[] 1 for counterclockwise; -1 for clockwise

%% propeller parameters
BLADES = 2;  %[] number of rotor blades
ROTOR = 4*0.0254;  %[m] rotor radius
CHORD = 17.55e-3;  %[m] mean aerodynamic chord
CHORD75 = 21e-3;  %[m] blade chord at 75% radius
ASLOPE = 5.9;  %[1/rad] blade lift curve slope (c_L over alpha)
Theta0 = 45*pi/180;  %[rad] theoretical blade pitch angle at rotor hub
THETT = -38.0973*pi/180;  %[rad] difference between blade pitch at 1) tip and 2) rotor hub
k_ind = 1.35;  %[] induced power correction factor => need to increase power by 35% to calc
a0 = 0;  %[rad] blade pre-cone angle => blades as rigigd bodies

%% frame parameters
Ax = 0.023;  %[m^2] equivalent flat plate drag area orthogonal to i_B
Ay = 0.023;  %[m^2] equivalent flat plate drag area orthogonal to j_B
Az = 0.106;  %[m^2] equivalent flat plate drag area orthogonal to k_B
STACP = 0;  %[m] stationline position of frame center of pressure
BLCP = 0;  %[m] buttline position of frame center of pressure
WLCP = -0.08;  %[m] waterline position of frame center of pressure

%% power plant parameters for DJI OPTO 30 ESC + DJI 2212 MOTOR
p00 = 7.145e-2;  % ESC+MOTOR System efficiency: eta_e = p00 + p10*Omega  +p01*Q + p20*Omega^2 + p11*Omega*Q + p02*Q^2;
p10 = 1.259e-3;
p01 = 0.4377;
p20 = -7.513e-7;
p11 = 1.284e-3;
p02 = -10.13;

n = 0.6359;  %exponent for ESC+Motor Model Omega=Omega_k*(PWM-PWM_idle)^n
k_Omega = 14.92;  %proportional parameter for ESC+Motor Model
PWM_idle = 1100; %[mus] idle value of PWM signal
PWM_max = 2000;  %[mus] max value of PWM signal


%% PX4 controller parameters

% velocity controller
g0 = 9.80665;  %[m/s^2] assumed gravity acceleration value for integral saturation limits
MPC_XY_VEL_P_ACC = 1.7*3/3;
MPC_Z_VEL_P_ACC  = 4;
MPC_XY_VEL_I_ACC = 0.8;
MPC_Z_VEL_I_ACC  = 2;
MPC_XY_VEL_D_ACC = 0.2*2;
MPC_Z_VEL_D_ACC  = 0;

% attitude controller
MC_ROLL_P = 3;
MC_PITCH_P = 3;
MC_YAW_P = 1.8*0.5;

% angular rate controller
MC_ROLLRATE_P = 0.09;
MC_ROLLRATE_I = 0.1*0.5;
MC_ROLLRATE_D = 0;   %if it works very well without dont use => derivate noisy signal
MC_PITCHRATE_P = 0.09;
MC_PITCHRATE_I = 0.1*0.5;
MC_PITCHRATE_D = 0;
MC_YAWRATE_P = 0.175*0.4;
MC_YAWRATE_I = 0.1*0;
MC_YAWRATE_D = 0;

% estimated trim condition
PWM_hover = (PWM_idle+PWM_max)/2;

% Speed mapping from gcs-input
MC_VEL_FWD = 5;
MC_VEL_LAT = 5;
MC_VEL_VERT = 1;

% rate mapping from gcs-input
MC_ATT_ROLL = 30*pi/180;
MC_ATT_PITCH = 30*pi/180;
MC_ATT_YAW = 180*pi/180;

% optical mapping
MC_VEL_FWD_OPT = 0.5;
MC_VEL_LAT_OPT = 0.5;
TERMINAL_SPEED = 0.5;


%% desired conditions
V_sin_x = 0.3;
Vsp = [0 1 0]';         %set point speed
del_sp = [0 0.7 0 0]';  %thrust roll pitch yaw => joystick input [-1,1] 

%% position of target
Xe_target = Xe_0 + [2, 3, -10]';




 