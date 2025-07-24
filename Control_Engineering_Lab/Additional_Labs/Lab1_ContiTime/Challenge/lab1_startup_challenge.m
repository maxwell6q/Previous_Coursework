% Matlab script to start the Simulink simulation of the blackbox model of 
% the Quanser SRV-02 + NI DAQ, using a robust state-space control approach.
% This script also evaluates the step response of the derived controller.
clear


%% Load Predefined Parameters
load_params_inertial_case


%% User Inputs
% Motor parameters (estimated from Motor 1)
mld.Beq = 1.3672e-6;     % [Nm/(rad/s)]
mld.tausf = 0.0148;      % [Nm]
mld.Jeq = 6.4640e-07;    % [kg m^2]

% Desired specifications
% Overshoot
specs.mp = 0.1; 
% Settling Time
specs.settling_time = 0.15; %[seconds]

% Simulation parameters
sIn.position = 70; 
sIn.simulation_time = 5; 

%% Reduced State-Space Model
% Plant Parameters
plant.km = (drv.dcgain*mot.Kt)/((mot.Req*mld.Beq) + (mot.Kt*mot.Ke));
plant.Tm = (mot.Req*mld.Jeq)/((mot.Req*mld.Beq) + (mot.Kt*mot.Ke));

% System Matrices
plant_SS.A = [0 1; 0 -(1/plant.Tm)];
plant_SS.B = [0 ;  plant.km/(gbox.N*plant.Tm)]; 
plant_SS.C = [1 0];
plant_SS.D = 0;


%% Extended StateSpace-Model
plant_SS.Ae = [0, plant_SS.C; zeros(2,1), plant_SS.A];
plant_SS.Be = [0; plant_SS.B];
plant_SS.Ce = [0, plant_SS.C];
plant_SS.De = plant_SS.D;

% Check if all eigenvalues of the closed loop can be freely allocated
% Reachability
plant_SS.R = ctrb(plant_SS.A, plant_SS.B);
% Zeros of plant tf in s=0
plant_SS.zeros = zero(ss(plant_SS.A, plant_SS.B, plant_SS.C, plant_SS.D));
if rank(plant_SS.R) < length(plant_SS.A) || any(plant_SS.zeros == 0)
    disp("WARNING: Eigenvalues of CL can not be freely allocated")
end


%% Eigenvalue Allocation
% Damping ratio from maximum overshoot
eig.damping = (log(1/specs.mp)) / (sqrt((pi^2) + (log(1/specs.mp)^2))); 

% Gain cut-off frequency from settling time 5% 
eig.wn = 3/(eig.damping*specs.settling_time); %[rad/s]

% real- and imaginary parts of eigenvalues
eig.real = -eig.damping*eig.wn; 
eig.img = eig.wn * sqrt(1 - eig.damping^2);

% Array of desired eigenvalues/poles
eig.values = [2*eig.real + 1i*eig.img, ...
              2*eig.real - 1i*eig.img, ...
              3*eig.real];

% Pole placement using acker
feedback.Ke = acker(plant_SS.Ae, plant_SS.Be, eig.values);

% State and Integrator Feedback
feedback.K  = feedback.Ke(2:end);
feedback.Ki = feedback.Ke(1);

% Nx(state feedforward gain) and Nu (input feedforward gain)
plant_SS.gains = ([plant_SS.A, plant_SS.B; ...
                   plant_SS.C, plant_SS.D]) \ [0;0;1];
feedback.Nx = plant_SS.gains(1:2);
feedback.Nu = plant_SS.gains(3);


%% Reduced State Observer / 2nd Order Butterworth Derivative
filt.omega_c = 2*pi*50; 
filt.delta = 1/sqrt(2);
filt.num = [filt.omega_c^2, 0];
filt.den = [1, 2*filt.delta*filt.omega_c, filt.omega_c^2];


%% Anti Windup and Tuning
feedback.Kw = 1/(specs.settling_time/2.05);
feedback.State_Tune = [2 , 0.5]; 
feedback.Int_Tune = 1.26;


%% Simulation and Results
sim("lab1_model_challenge.slx")
lab1_eval_challenge