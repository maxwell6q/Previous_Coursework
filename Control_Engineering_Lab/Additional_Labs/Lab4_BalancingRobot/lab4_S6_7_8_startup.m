% Startup file so as to estimate the state of the balancing robot from the
% measurements of the MPU and incremental encoder, and apply nominal LQR
% control using Bryson's rule. % There are two choices of gains in this 
% task set, with three different verifications to be run, it total there 
% are 2*3=6 results to be saved at the end of S6,7,8. 

clear 

% load selfmade model requirements with static parameters (balrob_params)
lab4_model_startup

%% Simulation parameters
% simulation parameters
% controller enable
sIn.ctrl = 1; 
sIn.Ts = 0.01;
sIn.simulation_time = 10; 
% swithced off by default
sIn.disturbance = 0;
% 1 for S81, 2 for S82, 3 for S83
sIn.case = 3; 
% 1 for rho equal to 500, 2 for rho equal to 5000, higher rho more
% contributions to cost function
sIn.feedback_gains = 1;

switch sIn.case
    case 1
        % Robot initial condition
        sIn.x0 =[ ...
        0, ... % gam(0)
        5*deg2rad, ... % th(0)
        0, ... % dot_gam(0)
        0]; % dot_th(0)

        % Blackboc initial condition
        sIn.bb_ic = [0, 0, 5*deg2rad, 0, 0, 0];

    case 2
        % Robot initial condition
        sIn.x0 =[ ...
        0, ... % gam(0)
        0, ... % th(0)
        0, ... % dot_gam(0)
        0]; % dot_th(0)

        % Blackboc initial condition
        sIn.bb_ic = [0, 0, 0, 0, 0, 0];

    case 3
        % Robot initial condition
        sIn.x0 =[ ...
        0, ... % gam(0)
        0, ... % th(0)
        0, ... % dot_gam(0)
        0]; % dot_th(0)

        % Blackboc initial condition
        sIn.bb_ic = [0, 0, 0, 0, 0, 0];
        % disturbance is on
        sIn.disturbance = 1;
        % longer simulation time to see the effect of disturbance at t=10
        sIn.simulation_time = 20; 

end



%% Tilt angle estimation 
% Tilt angle estimation via a complementary filter approach
% Low-pass filter for the accelerometer, high-pass for the gyroscope
% Their sum is valid on the entire frequency range

% Low pass filter cutoff frequency 
filter.low.fc = 0.35; 
% Low pass filter time constant 
filter.low.Tc = 1/(2*pi*filter.low.fc); 
% First order low pass filter
filter.low.firstorderCT = tf(1, [filter.low.Tc 1]);
% Strictly proper discrete time filter
filter.low.firstorderDT = c2d(filter.low.firstorderCT, sIn.Ts, 'zoh');

% High pass filter ("real derivative")
% as mentioned in eqn 7 in assignment
filter.high.N = 3; 
filter.high = tf([1 0 0 -1], [filter.high.N*sIn.Ts, 0, 0, 0], sIn.Ts);

%% DT SS control based on LQR using Bryson's rule
% The following methodology should gurantee nominal perfect tracking of
% constant wheel angle position set-point gamma

%% Continous time plant model 
% with the state x=[gamma, theta, dot_gamma, dot_theta]
tmp.m11 = 2*wheel.Iyy + 2*(gbox.N^2)*(mot.rot.Iyy) + ...
    (body.m + 2*wheel.m + 2*mot.rot.m)*(wheel.r^2);

% same for m21
tmp.m12 = 2*gbox.N*(1 - gbox.N)*mot.rot.Iyy + ...
    (body.m*body.zb + 2*mot.rot.m*mot.rot.zb)*wheel.r;

tmp.m22 = body.Iyy + 2*((1 - gbox.N)^2)*(mot.rot.Iyy) + ...
    body.m*(body.zb^2) + 2*mot.rot.m*((mot.rot.zb)^2);

%eqn 45 in methods
tmp.M = [tmp.m11, tmp.m12; tmp.m12, tmp.m22]; 

tmp.g22 = -(body.m*body.zb + 2*mot.rot.m*mot.rot.zb)*g; 

tmp.Fv = [2*(gbox.B + wheel.B) -2*gbox.B; -2*gbox.B 2*gbox.B];
tmp.Fv_const = (2*(gbox.N^2)*mot.Kt*mot.Ke)/mot.R; 
tmp.Fv_hat = tmp.Fv + tmp.Fv_const*[1 -1; -1 1];

tmp.G = [0 0; 0 tmp.g22]; 

% eqn 58 in methods
plant.A = [zeros(2,2), eye(2,2); 
          -inv(tmp.M)*tmp.G, -inv(tmp.M)*tmp.Fv_hat];

plant.B = ((2*gbox.N*mot.Kt)/mot.R)*([zeros(2,2); inv(tmp.M)]*[1;-1]);
plant.C = [1 0 0 0]; 
plant.D = 0; 

% create a ss model
plant.ssCT = ss(plant.A, plant.B, plant.C, plant.D); 

%% Discretize the model with exact discretization 
plant.ssDT = c2d(plant.ssCT, sIn.Ts, 'zoh'); 
plant.gainsDT = ([plant.ssDT.A - eye(4), plant.ssDT.B; plant.ssDT.C, plant.ssDT.D])\[flip(eye(5,1))];
feedback.Nx = plant.gainsDT(1:end-1); 
feedback.Nu = plant.gainsDT(end); 

%% LQR using Bryson's rule
% Q commands the control accuracy, and R the control effort
% the extra factor rho is used to consider relative weighting between 
% control input and the control energies 
LQR.rho = [500, 5000]; 
% eqn 14 in assignment
LQR.gamma_bar = pi/18; 
LQR.theta_bar = pi/360; 
LQR.u_bar = 1; 

LQR.Q = diag([1/((LQR.gamma_bar)^2), 1/((LQR.theta_bar)^2), 0, 0]); 
LQR.R = (1/(LQR.u_bar^2)); 
% with both values of rho

switch sIn.feedback_gains
    case 1
        feedback.K1 = dlqr(plant.ssDT.A, plant.ssDT.B, LQR.Q, LQR.R*LQR.rho(1));
    case 2
        feedback.K2 = dlqr(plant.ssDT.A, plant.ssDT.B, LQR.Q, LQR.R*LQR.rho(2));
end















