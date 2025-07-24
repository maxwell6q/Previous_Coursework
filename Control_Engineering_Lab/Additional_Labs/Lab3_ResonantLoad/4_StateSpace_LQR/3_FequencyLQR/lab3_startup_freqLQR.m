% Matlab script to start the Simulink simulation of an accurate model of 
% the Quanser SRV-02 + NI DAQ with a resonant load and a position state-
% space controller with chosen eigenvalues.
clear



%% Load Predefined Parameters
load_params_resonant_case

sIn.motor_or_nominal = 1;   % 0: motor;  1: nominal
load_params_model 


%% User Inputs
sIn.intOn = 0;     % 0: nominal,  1: robust
LQR.q22 = 100;     % weight for resonant frequency
LQR.qi = 10;     % weight for integrator

% Desired specifications
% Overshoot
specs.mp = 0.3; % [30%] 
% Settling Time
specs.settling_time = 0.5; % [s]



%% Simulation Parameters 
% Solver step time (0.1 ms)
sIn.solver_time = 1e-4;

% List of reference positions [deg]
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

plant.sys = ss(plant.A,plant.B,plant.C,plant.D);


%% Frequency based LQR controller
% Acceptable deviation of state
LQR.bar_theta_h = 0.3*sIn.position*pi/180;

% Actuation weight
LQR.bar_u = 10; %[v]
LQR.R=(1/LQR.bar_u^2);

% Identify the resonant frequency of the c.c. pole pair
[plant.wn, plant.damp] = damp(plant.sys);
for i=1:length(plant.A)
    if abs(plant.damp(i)) < 1
        plant.w0 = plant.wn(i);
        continue
    end
end

% Factorization of the weight
% q22(w) = H'*Q(jw)H'Q(jw);  H'Q(jw) = sqrt(q22) w0^2/(s^2+w0^2)

% State-space realization of H'Q(jw)
LQR.AQ_prime = [0,1; -plant.w0^2, 0];
LQR.BQ_prime = [0; 1];
LQR.CQ_prime = [sqrt(LQR.q22)*plant.w0^2, 0];
LQR.DQ_prime = 0;

% Factorization of the weight matrix
% Q(jw) = H*Q(jw)HQ(jw);  HQ(jw) = diag(1/bar{th_h}, H'Q(jw), 0, 0)

% State-space realization of HQ(s)
LQR.AQ = LQR.AQ_prime;
LQR.BQ = [zeros(2,1), LQR.BQ_prime, zeros(2,2)];
LQR.CQ = [zeros(1,2); LQR.CQ_prime; zeros(2,2)];
LQR.DQ = diag([1/LQR.bar_theta_h, LQR.DQ_prime, 0, 0]);

% Augmented state model
plant.AA = [plant.A, zeros(4,2); LQR.BQ, LQR.AQ];
plant.BA = [plant.B; zeros(2,1)];
plant.CA = [plant.C, zeros(1,2)];
plant.DA = 0;

% Augmented cost matrices
LQR.QA = [LQR.DQ'*LQR.DQ, LQR.DQ'*LQR.CQ;...
          LQR.CQ'*LQR.DQ, LQR.CQ'*LQR.CQ];
LQR.RA = LQR.R;

% Feedback matrix
feedback.K = lqr(plant.AA, plant.BA, LQR.QA, LQR.RA);

% State feedforward gain and input feedforward gain
feedback.gains = ([plant.AA, plant.BA; plant.CA, plant.DA])\[zeros(6,1);1];
feedback.Nx = feedback.gains(1:end-1);
feedback.Nu = feedback.gains(end);


%% Integrator augmentation
% Extended statespace model
plant.Ae = [0, plant.CA; zeros(6,1), plant.AA];
plant.Be = [0;plant.BA];
plant.Ce = [0,plant.CA];

% Extended cost matrices
LQR.Qe = [LQR.qi, zeros(1,6); zeros(6,1), LQR.QA];
LQR.Re = LQR.RA;

% Feedback matrix
feedback.robustKe = lqr(plant.Ae, plant.Be, LQR.Qe, LQR.Re);
feedback.robustKi = feedback.robustKe(1);
feedback.robustK  = feedback.robustKe(2:end);

if sIn.intOn == 1
    feedback.K = feedback.robustK;
end



%% Simple Observer
filt.wc = 2*pi*50;
filt.del = 1/sqrt(2);
filt.num = [filt.wc^2, 0];
filt.den = [1, 2*filt.del*filt.wc, filt.wc^2];



%% Run Simulation
%set_param('lab3_model_freqLQR', 'AlgebraicLoopMsg', 'none');
%simOut = sim('lab3_model_freqLQR');
%evalLQR