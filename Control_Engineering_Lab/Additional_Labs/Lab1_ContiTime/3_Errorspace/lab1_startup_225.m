% Matlab script to start the Simulink simulation of the blackbox- and real
% model of the Quanser SRV-02 + NI DAQ. In both cases state-space feedback
% is used. 
clear



%% Load Predefined Parameters

load_params_inertial_case



%% User Inputs

% Desired specifications
specs.mp = 0.1; 
specs.settling_time = 0.15; %[seconds]


% Validation Task (6, 7 or 8)
sIn.val = 6;
%Amplitude of reference signal (only for 6)
sIn.Amp = 30;



%% Inputs for simulation 
switch sIn.val
    case 6
        sIn.T_r = 0.5;
        sIn.Freq = (1/sIn.T_r)*2*pi; 
        sIn.program = 1;
        sIn.simulation_time = sIn.T_r*10;
    case 7
        sIn.T_r = 0.1;
        sIn.Freq = (1/sIn.T_r)*2*pi; 
        sIn.Amp = 40;
        sIn.program = 1;
        sIn.simulation_time = sIn.T_r*10;
    case 8
        sIn.Amp = 40;
        sIn.program = 2;
        sIn.simulation_time = 5;
        sIn.Freq = 1;%(just for Simulink completeness)
end

%Frequency of the sin wave to track
specs.T_r = 0.5;
specs.Freq = (1/specs.T_r)*2*pi; 


%% Motor Parameters

% Nominal Parameters (estimated from Blackbox)
%%mld.Beq = 1.2224e-6;    % [Nm/(rad/sec)] 
%%mld.tausf = 0.0056;     % [Nm]
%%mld.Jeq = mld.Jeq;      % [kg m^2]

% Actual Parameters (estimated from Motor 1)
mld.Beq = 1.3672e-6;     % [Nm/(rad/s)]
mld.tausf = 0.0148;      % [Nm]
mld.Jeq = 6.4640e-07;    % [kg m^2]


% High-pass filter (Continous - Time) for load velocity calculation
filt.high.omega_c = 2*pi*50; 
filt.high.delta = 1/sqrt(2);
filt.high.num = [filt.high.omega_c^2, 0];
filt.high.den = [1, 2*filt.high.delta*filt.high.omega_c, filt.high.omega_c^2];


%% Plant Parameters

% reduced plant transfer function
plant.km = (drv.dcgain*mot.Kt)/((mot.Req*mld.Beq) + (mot.Kt*mot.Ke));
plant.Tm = (mot.Req*mld.Jeq)/((mot.Req*mld.Beq) + (mot.Kt*mot.Ke));



% Calculation of damping ratio from maximum overshoot
eig.damping = (log(1/specs.mp)) / (sqrt((pi^2) + (log(1/specs.mp)^2))); 
% calculation of gain cut-off frequency from settling time 5% 
eig.W_n = 3/(eig.damping*specs.settling_time); %[rad/s]



% Plant's A, B, C, D matrix 
plant_SS.A = [0 1; 0 -(1/plant.Tm)];
plant_SS.B = [0 ;  plant.km/(gbox.N*plant.Tm)]; 
plant_SS.C = [1 0];
plant_SS.D = 0;

% Exo–system for reference signal with constant disturbance
ref_SS.A_r = [0 1 0; 0 0 1 ; 0 -specs.Freq^2 0];
ref_SS.C_r = [1 0 0];


% Submatrices for combined statespace
eig.A_12 = [0 0;0 0; plant_SS.C];
eig.A_21 = zeros([size(plant_SS.A,1),size(ref_SS.A_r,2)]);
eig.B_12 = zeros([size(ref_SS.A_r,1),size(plant_SS.B,2)]);

% Combined statespace matrices
hat_SS.A = [ref_SS.A_r eig.A_12; eig.A_21 plant_SS.A];
hat_SS.B = [eig.B_12;plant_SS.B];



%% Eigenvalues
% First conjugate pair (-pi ± pi/4)
[math.x1, math.y1] = pol2cart(-pi+pi/4, eig.W_n);
[math.x2, math.y2] = pol2cart(-pi-pi/4, eig.W_n);

% Second conjugate pair (-pi ± pi/6)
[math.x3, math.y3] = pol2cart(-pi+pi/6, eig.W_n);
[math.x4, math.y4] = pol2cart(-pi-pi/6, eig.W_n);

eig.values = [math.x1 + 1i*math.y1, math.x1 - 1i*math.y1, ...
              math.x3 + 1i*math.y3, math.x3 - 1i*math.y3, ...
              -eig.W_n];


% calculation of feedback via acker and place
feedback.K= place(hat_SS.A, hat_SS.B, eig.values);





