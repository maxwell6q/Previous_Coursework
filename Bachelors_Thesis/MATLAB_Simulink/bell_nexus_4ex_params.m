%% Script to Define the Parameters for the Bell Nexus 4EX
% this includes model parameters (system and typical winds)
% and display and plot parameters
% simulation parameters are set in bell_nexus_4ex.m script

%   Ver.      Date        Author
%   1.0    24.07.2024    M. Pries
%--------------------------------------



%% Definition of the Parameters for both Trim Conditions
% state space matrices
A_lon1 = [-0.0382   0.0724      0     -9.8100
          -0.2943  -0.7920   65.1513      0
              0    -0.0095   -0.6353      0
              0        0      1.000       0];

B_lon1 = [   0
          -8.7116
           3.3176
             0];

A_lat1 = [-0.1145   0.0924  -66.4142  9.8100
          -0.0193  -2.3771    0.0461     0
           0.0258  -0.3963   -0.1239     0
              0      1.000       0       0];

B_lat1 = [-6.3953     0
          -3.6988  28.4999
           5.0463   2.3744
              0       0];

A_lon2 = [-0.0517   0.0900     0     -9.8100
          -0.3658  -0.7024  52.2119      0
              0    -0.0228  -0.6811      0
              0         0    1.0000      0];

B_lon2 = [   0
          -6.2164
           2.7890
             0];

A_lat2 = [-0.0921  -0.0744  -53.4342  9.8100
          -0.0144  -1.9290    0.0559     0
           0.0206  -0.5032   -0.0979     0
           0        1.0000      0        0];

B_lat2 = [-4.5635    0
          -1.1832  9.2734
           2.1759  0.8010
              0       0];

C_lon = eye(4);
D_lon = zeros(4,1);

C_lat = eye(4);
D_lat = zeros(4,2);

% reference cruise speeds [m/s]
u01 = 120*0.44704;
u02 = 150*0.44704;

% packing the data
tc_ = {A_lon1, B_lon1, A_lat1, B_lat1, u01;
       A_lon2, B_lon2, A_lat2, B_lat2, u02};



%% Wind Transfer Matrices
Tw_lon = [1 0 0
         0 0 1
         0 0 0
         0 0 0];

Tw_lat = [0 1 0
         zeros(3)];



%% PID Controller Parameters
deriv_lag = 0.05;

% elevator (from paper and new)
kp_elevator_paper = 50;
ki_elevator_paper = 20;
kd_elevator_paper = 30;

kp_elevator_new = 2.771;
ki_elevator_new = 1.700;
kd_elevator_new = 0.7293;

elevator_ = {kp_elevator_new,   ki_elevator_new,   kd_elevator_new;
             kp_elevator_paper, ki_elevator_paper, kd_elevator_paper};

% rudder (only paper)
kp_rudder = 0.5;
ki_rudder = 0;
kd_rudder = 0;

% aileron (from paper and new)
kp_aileron_paper = 2;
ki_aileron_paper = 1;
kd_aileron_paper = 1;

kp_aileron_new = 0.2926;
ki_aileron_new = 0.2200;
kd_aileron_new = 0.0726;

aileron_ = {kp_aileron_new,   ki_aileron_new,   kd_aileron_new;
            kp_aileron_paper, ki_aileron_paper, kd_aileron_paper};



%% Prefilter Parameters
% elevator time constant [s]
elev_pf_tau = 0.7;



%% Actuator Parameters
% time constant [s]
tau = 0.1;

% deflection limits [deg]/[deg/s]
elev_lim = [-15 22];
rudder_lim = [-25 25];
ailer_lim = [-15 20];
rate_lim = 30;



%% Wind as Sinusoid
% frequencies for modes [rad/s]
omega = [1.07 3.14 5.22]; 

% amplitudes for modes [m/s]
a_u = [3.21 1.07 0.75];
a_v = [3.89 0.82 0.58]; 
a_w = [2.49 0.84 0.61];



%% Display Parameters
trim_disp = {'Trim Condition 1 (150mph)', 'Trim Condition 2 (120mph)'};

sys_disp = {'Detailed System', '';
            'Only Actuator'  , 'Only Derivative Lag';
            ' '              , 'Reduced System'};

pf_disp = {'With Pre-filter on Elevator', 'No Pre-Filter on Elevator'};

con_disp = {'New Controller', 'Controller from Literature'};

wind_disp = {'Wind as Step', 'Urban Wind Conditions';
             'Lon: ', 'Lat: ';
             '', ''};

sp_disp = {' Zero-Setpoint', ' Setpoint: '};



%% Plot Parameters
% set default line width
set(groot,'DefaultLineLineWidth',1.5)

% set default a
set(groot,'DefaultAxesFontSize',14)