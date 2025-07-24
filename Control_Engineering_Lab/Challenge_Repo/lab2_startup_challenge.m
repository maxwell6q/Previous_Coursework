% Matlab script to start the Simulink simulation an accurate model of the 
% Quanser SRV-02 + NI DAQ with a discrete time state-space controller and
% a reduced state observer.
clear



%% Load Predefined Parameters
load_params_inertial_case



%% User Inputs
% Sampling time
sIn.T_s =200e-3; %[s]

% Solver step time (0.1 ms)
sIn.step_size = 1e-4;

% Reference position [s]
sIn.position = 60;

% Simulation time [s]
sIn.simulation_time = 2;



%% State-Space Model
% Motor Parameters (estimated from Motor 1)
mld.Beq = 2.5663e-6;    % [Nm/(rad/s)]
mld.tausf = 0.013;      % [Nm]
mld.Jeq = 3.4640e-07;   % [kg m^2]

% Plant Parameters
plant.km = (drv.dcgain*mot.Kt)/((mot.Req*mld.Beq) + (mot.Kt*mot.Ke));
plant.Tm = (mot.Req*mld.Jeq)/((mot.Req*mld.Beq) + (mot.Kt*mot.Ke));

% System matrices
plant.A = [0, 1;   0, -(1/plant.Tm)];
plant.B = [0; plant.km/(gbox.N*plant.Tm)]; 
plant.C = [1, 0];
plant.D = 0;

% Extended state-space model
plant.Ae = [0, plant.C; zeros(2,1), plant.A];
plant.Be = [0; plant.B];
plant.Ce = [0, plant.C];
plant.De = plant.D;



%% Feedback Calculations
% State feedforward gain and input feedforward gain
feedback.gains = ([plant.A, plant.B; plant.C, plant.D])\[0;0;1];
feedback.Nx = feedback.gains(1:2);
feedback.Nu = feedback.gains(3);

% Measured rarameters from impulse response (Ts=0.20s)
impVal = 1.355;
thErr1 = 0.09844;
omega1 = 26.04;

% Desired feedback gains to reproduce the impulse
feedback.integrator = 0.3;

feedback.k1 = impVal/(sIn.position*deg2rad);
feedback.k2 = (thErr1*feedback.k1 + ...
               feedback.integrator*sIn.position*deg2rad)/omega1;

feedback.state = [feedback.k1, feedback.k2];

% Stability check
eig.values = eig(plant.Ae-plant.Be*[feedback.integrator, feedback.state]);
if (any(eig.values>=0))
    disp("WARNING: Unstable feedback system!")
end



%% Reduced Observer Model
% Observer eigenvalue - Continuous Time
eigO.value = 13.23*eig.values(1);

% Observer gain
obs.L = acker(plant.A(2,2), plant.A(1,2), eigO.value);

% Observer matrices - CT (simpilifcations applied)
obs.A0 = plant.A(2,2)-obs.L*plant.A(1,2);
obs.B0 = [plant.B(2), obs.A0*obs.L];
obs.C0 = [0; 1];
obs.D0 = [0, 1;  0, obs.L];

% Discretized Observer - DT Backward Euler
obs.Phi0 = 1/(1-obs.A0*sIn.T_s);
obs.Gamma0 = 1/(1-obs.A0*sIn.T_s) *obs.B0*sIn.T_s;
obs.H0 = obs.C0/(1-obs.A0*sIn.T_s);
obs.J0 = obs.D0 + obs.C0/(1-obs.A0*sIn.T_s) *obs.B0*sIn.T_s;



%% Simulation and Results
sim("lab2_model_challenge.slx");
lab2_eval_challenge