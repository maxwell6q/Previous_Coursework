% Matlab script to start the Simulink simulation of the blackbox- and real
% model of the Quanser SRV-02 + NI DAQ. In both cases state-space feedback
% is used. 
clear



%% Load Predefined Parameters
load_params_inertial_case



%% User Inputs

% Motor Parameters
% Nominal Parameters (estimated from Blackbox)
%mld.Beq = 1.2224e-6;    % [Nm/(rad/sec)] 
%mld.tausf = 0.0056;     % [Nm]
%mld.Jeq = mld.Jeq;      % [kg m^2]

% Actual Parameters (estimated from Motor 1)
mld.Beq = 1.3672e-6;     % [Nm/(rad/s)]
mld.tausf = 0.0148;      % [Nm]
mld.Jeq = 6.4640e-07;    % [kg m^2]



% Set of eigenvalues
eig.set = "3";

%% Plant parameters Parameters

% reduced plant transfer function
plant.km = (drv.dcgain*mot.Kt)/((mot.Req*mld.Beq) + (mot.Kt*mot.Ke));
plant.Tm = (mot.Req*mld.Jeq)/((mot.Req*mld.Beq) + (mot.Kt*mot.Ke));


%% State feedback matrix with integrator calculation [2.2.3]  
% High-pass filter (Continous - Time) for load velocity calculation
filt.high.omega_c = 2*pi*50; 
filt.high.delta = 1/sqrt(2);
filt.high.num = [filt.high.omega_c^2, 0];
filt.high.den = [1, 2*filt.high.delta*filt.high.omega_c, filt.high.omega_c^2];


% Desired specifications
specs.mp = 0.1; 
specs.settling_time = 0.15; %[seconds]

% Plant's A, B, C, D matrix 
plant_SS.A = [0 1; 0 -(1/plant.Tm)];
plant_SS.B = [0 ;  plant.km/(gbox.N*plant.Tm)]; 
plant_SS.C = [1 0];
plant_SS.D = 0;

% Extended StateSpace-Model
plant_SS.Ae = [0, plant_SS.C; zeros(2,1), plant_SS.A];
plant_SS.Be = [0; plant_SS.B];
plant_SS.Ce = [0, plant_SS.C];
plant_SS.De = plant_SS.D;

% Check if all eigenvalues of the closed loop can be freely allocated
% Reachability
plant_SS.R = ctrb(plant_SS.A, plant_SS.B);
plant_SS.zeros = zero(ss(plant_SS.A, plant_SS.B, plant_SS.C, plant_SS.D));
if rank(plant_SS.R) < length(plant_SS.A) || any(plant_SS.zeros == 0)
    disp("WARNING: Eigenvalues of CL can not be freely allocated")
end


% desired closed loop eigenvalues 
% Calculation of damping ratio from maximum overshoot
eig.damping = (log(1/specs.mp)) / (sqrt((pi^2) + (log(1/specs.mp)^2))); 
% calculation of gain cut-off frequency from settling time 5% 
eig.wn = 3/(eig.damping*specs.settling_time); %[rad/s]


% real- and imaginary parts of eigenvalues (poles calculation)
eig.real = -eig.damping*eig.wn; 
eig.img = eig.wn * sqrt(1 - eig.damping^2);


% eigenvalues for nominal CL
eig.nominal.values = [eig.real + 1i*eig.img, ...
                      eig.real - 1i*eig.img];

% four different sets of eigenvalues for integral augmented CL
eig.int.values1 = [eig.real + 1i*eig.img, ...
                   eig.real - 1i*eig.img, ...
                   eig.real];

eig.int.values2 = [eig.real, ...
                   eig.real,...
                   eig.real];

eig.int.values3 = [2*eig.real + 1i*eig.img, ...
                   2*eig.real - 1i*eig.img, ...
                   2*eig.real];

eig.int.values4 = [2*eig.real + 1i*eig.img, ...
                   2*eig.real - 1i*eig.img, ...
                   3*eig.real];


% calculation of feedback via acker and place
% The second set of evals has multiplicity 3 > rank(B) => place dont work
%[feedback.int.place.Ke, feedback.int.place.prec] =...
%    place(plant_SS.Ae, plant_SS.Be, eig.int.("values"+eig.set));
%feedback.int.place.Ki = feedback.int.place.Ke(1);
%feedback.int.place.K  = feedback.int.place.Ke(2:end);

feedback.integ.acker.Ke = ...
    acker(plant_SS.Ae, plant_SS.Be, eig.int.("values"+eig.set));
feedback.integ.acker.Ki = feedback.integ.acker.Ke(1);
feedback.integ.acker.K  = feedback.integ.acker.Ke(2:end);

% calculation of feedback for the nominal case
[feedback.nominal.place.K, feedback.nominal.place.prec] =...
    place(plant_SS.A, plant_SS.B, eig.nominal.values);


% calculation of Nx(state feedforward gain), Nu (input feedforward gain)
plant_SS.gains = ([plant_SS.A, plant_SS.B; plant_SS.C, plant_SS.D])\[0;0;1];
feedback.Nx = plant_SS.gains(1:2);
feedback.Nu = plant_SS.gains(3);


%% Simulation Parameters 

% Define the time and output values
sIn.position = [0, 40, 0, 70, 0, 120, 0]; 
sIn.num = length(sIn.position);
sIn.sample_time = 5; 
sIn.simulation_time = sIn.sample_time*length(sIn.position) - 1;