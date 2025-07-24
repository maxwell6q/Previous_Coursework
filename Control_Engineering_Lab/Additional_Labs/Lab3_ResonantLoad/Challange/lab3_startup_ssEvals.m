% Matlab script to start the Simulink simulation of an accurate model of 
% the Quanser SRV-02 + NI DAQ with a resonant load and a position state-
% space controller with chosen eigenvalues, optimized for settling time.
clear


%% Load Predefined Parameters
load_params_resonant_case

sIn.motor_or_nominal = 0;   % 0: motor;  1: nominal
load_params_model


%% User Inputs
% Run the simulation?
run_sim = 0;

% Desired overshoot
specs.mp = 0.21;

% Desired Settling Time
specs.settling_time = 0.11; % [s]

% Phase difference between the two cc. pole pairs
eigP.phaseTune = 2.89;

% Magnitude difference between the two cc. pole pairs
eigP.magTune = 0.44;

% Integrator pole tuning
eigP.intTune = 0.96;

% Anti Windup gain
feedback.AWgain = 33;


%% Simulation Parameters 
% Solver step time (0.1 ms)
sIn.solver_time = 1e-4;

% List of reference positions [s]
sIn.position = 50; 

% Time the reference positions are held [s]
sIn.step_time = 2; 

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


% State feedforward gain and input feedforward gain
feedback.gains = ([plant.A, plant.B; plant.C, plant.D])\[zeros(4,1);1];
feedback.Nx = feedback.gains(1:end-1);
feedback.Nu = feedback.gains(end);



%% Feedback Controller Design
% Desired dynamic parameters for approximation
eigP.damping = log(1/specs.mp) / sqrt(pi^2 + log(1/specs.mp)^2);
eigP.wn = 3/(eigP.damping*specs.settling_time);

% Phase of eigenvalues
eigP.phi = atan2(sqrt(1-eigP.damping^2), eigP.damping);

% Desired eigenvalues for nominal tracking
eigP.values = eigP.wn*[exp(1i*(-pi+eigP.phi)), ...
          eigP.magTune*exp(1i*(-pi+eigP.phi/eigP.phaseTune))];
eigP.values = [eigP.values, conj(eigP.values)];

% Eigenvalues for robust tracking
eigP.robustValues = [eigP.values, -eigP.intTune*eigP.wn];

% Extended statespace model
plant.Ae = [0, plant.C; zeros(4,1), plant.A];
plant.Be = [0;plant.B];
plant.Ce = [0,plant.C];


% State feedback matrix frot the robust case
feedback.robustKe = place(plant.Ae, plant.Be, eigP.robustValues);
feedback.robustKi = feedback.robustKe(1);
feedback.robustK  = feedback.robustKe(2:end);


%% Simple Observer
filt.wc = 2*pi*50;
filt.del = 1/sqrt(2);
filt.num = [filt.wc^2, 0];
filt.den = [1, 2*filt.del*filt.wc, filt.wc^2];


%% Simulation
if run_sim == 1
    set_param('lab3_ssEvals', 'AlgebraicLoopMsg', 'none');
    simOut = sim('lab3_ssEvals.slx');
    evalLQR
end