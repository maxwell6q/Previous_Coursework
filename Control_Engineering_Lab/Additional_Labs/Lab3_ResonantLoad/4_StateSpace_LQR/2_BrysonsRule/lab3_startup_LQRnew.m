% Matlab script to start the Simulink simulation of an accurate model of 
% the Quanser SRV-02 + NI DAQ with a resonant load and a position state-
% space controller with chosen eigenvalues.
clear



%% Load Predefined Parameters
load_params_resonant_case

sIn.motor_or_nominal = 1;   % 0: motor;  1: nominal
load_params_model


%% User Inputs
sIn.intOn = 0;  % 1: nominal;  0: robust

q11 = 0.01;
% Desired specifications
% Overshoot
specs.mp = 0.3; % [30%] 
% Settling Time
specs.settling_time = 0.5; % [s]



%% Simulation Parameters 
% Solver step time (0.1 ms)
sIn.solver_time = 1e-4;

% List of reference positions [s]
sIn.position = 50; 

% Time the reference positions are held [s]
sIn.step_time = 3; 

% Automatic calculation of total simulation time [s]
sIn.simulation_time = sIn.step_time*length(sIn.position);

% Time for averaging the bias of displacement sensor
sIn.t0 = 0.2;
sIn.t1 = 0.7;


%% State-Space Model
% with the state x=[theta_h, theta_d, omega_h, omega_d]
tmp.v1 = 1 /(gbox.N^2 * mld.Jeq);
tmp.v2 = (mld.Beq + mot.Kt*mot.Ke/mot.Req) /mld.Jeq;
tmp.v3 = mot.Kt*drv.dcgain /gbox.N /mld.Jeq / mot.Req;

plant.A = [zeros(2,2),    eye(2,2);
      0,  mld.k*tmp.v1, -tmp.v2, 0;
      0, -mld.k/mld.Jb-mld.k*tmp.v1, tmp.v2-mld.Bb/mld.Jb, -mld.Bb/mld.Jb];

plant.B = [0;0; tmp.v3; -tmp.v3];

plant.C = [1,0,0,0];
plant.D = 0;

sys= ss(plant.A,plant.B,plant.C,plant.D);



%% Feedback Controller Design
% Desired dynamic parameters for approximation
eigP.damping = log(1/specs.mp) / sqrt(pi^2 + log(1/specs.mp)^2);
eigP.wn = 3/(eigP.damping*specs.settling_time);


%% LQR control 
LQR.bar_theta_h = 0.3*sIn.position*pi/180;
LQR.bar_theta_d = pi/36; 
LQR.bar_u = 10; %[v]
LQR.Q = diag([1/LQR.bar_theta_h^2 1/LQR.bar_theta_d^2 0 0]);
LQR.R=(1/LQR.bar_u^2);


feedback.K = lqr(sys, LQR.Q, LQR.R);

% State feedforward gain and input feedforward gain
feedback.gains = ([plant.A, plant.B; plant.C, plant.D])\[zeros(4,1);1];
feedback.Nx = feedback.gains(1:end-1);
feedback.Nu = feedback.gains(end);

%% Robust Statespace
% Extended statespace model
plant.Ae = [0, plant.C; zeros(4,1), plant.A];
plant.Be = [0;plant.B];
plant.Ce = [0,plant.C];
syse = ss(plant.Ae,plant.Be,plant.Ce,plant.D);
LQR.Qe = diag([q11 1/LQR.bar_theta_h^2 1/LQR.bar_theta_d^2 0 0]);
feedback.robustK = lqr(syse, LQR.Qe, LQR.R);
feedback.robustKi = feedback.robustK(1);
feedback.robustK  = feedback.robustK(2:end);


%% Simple Observer
filt.wc = 2*pi*50;
filt.del = 1/sqrt(2);
filt.num = [filt.wc^2, 0];
filt.den = [1, 2*filt.del*filt.wc, filt.wc^2];


