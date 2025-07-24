% Matlab script to start the Simulink simulation of an accurate model of 
% the Quanser SRV-02 + NI DAQ with a discrete time PID controller and its 
% improvements such as feedforward and antiwindup.
clear

%% Load Predefined Parameters
load_params_inertial_case

%% User Inputs

% Actual Parameters (estimated from Motor 1)
mld.Beq = 2.5663e-6;    % [Nm/(rad/s)]
mld.tausf = 0.013;      % [Nm]
mld.Jeq = 3.4640e-07;   % [kg m^2]

% Desired specifications
% Overshoot
specs.mp = 0.1; 
% Settling Time
specs.settling_time = 0.15; %[s]

%% PID Parameters

% reduced plant transfer function
plant.km = (drv.dcgain*mot.Kt)/((mot.Req*mld.Beq) + (mot.Kt*mot.Ke));
plant.Tm = (mot.Req*mld.Jeq)/((mot.Req*mld.Beq) + (mot.Kt*mot.Ke));
plant.Ps = tf(plant.km, [(gbox.N*plant.Tm) gbox.N 0]); 

% resulting gains from bode method
%PID = computePIDGains(8, specs.settling_time, specs.mp, plant.Ps, "PID");
PID.Kp = 7.845;
PID.Ki = 100.8347;
PID.Kd = 0.0763;
PID.Tl = 700e-04;
PID.Cs = pid(PID.Kp, PID.Ki, PID.Kd, PID.Tl); 

% Anit Windup
PID.t_s5 = 0.15; % 5% settling time from lab0
PID.Kw = 1/(PID.t_s5/4.5); % anit windup gain: 1/Tw, Tw=t_s5/5


%% Simulation Parameters 
% Sampling Time Vector [1 ms 10ms 50ms]
specs.Ts = [1e-3; 1e-2; 5e-2]; % [s]
% Step reference input
sIn.position = [50];
sIn.simulation_time = 5;



%% Discrete-Time PID
% Define discretization methods and their corresponding labels
methods = {'BackwardEuler', 'ForwardEuler', 'Trapezoidal'};
methodLabels = {'BE', 'FE', 'Tustin', 'zoh'};

% Initialize storage structures
for m = 1:length(methods)
    label = methodLabels{m};
    PID.Discrete.(label).Cz = cell(length(specs.Ts), 1);
    PID.Discrete.(label).num = cell(length(specs.Ts), 1);
    PID.Discrete.(label).den = cell(length(specs.Ts), 1);
end

% Loop over each sample time and discretization method
for i = 1:length(specs.Ts)
    Ts = specs.Ts(i);
    
    for m = 1:length(methods)
        method = methods{m};
        label = methodLabels{m};
        
        % Create discrete-time PID controller
        PID.Discrete.(label).Cz{i} = pid(PID.Kp, PID.Ki, PID.Kd, PID.Tl,...
            Ts, 'IFormula', method, 'DFormula', method);
        
        % Extract numerator and denominator coefficients
        [PID.Discrete.(label).num{i}, PID.Discrete.(label).den{i}] = ...
            tfdata(PID.Discrete.(label).Cz{i}, 'v');
    end
end

%% ZOH
PID.Discrete.ZOH1 = c2d(PID.Cs, specs.Ts(1), 'zoh');
PID.Discrete.ZOH10 = c2d(PID.Cs, specs.Ts(2), 'zoh');
PID.Discrete.ZOH50 = c2d(PID.Cs, specs.Ts(3), 'zoh');

[numZOH1, denZOH1] = tfdata(PID.Discrete.ZOH1, 'v');
[numZOH10, denZOH10] = tfdata(PID.Discrete.ZOH10, 'v');
[numZOH50, denZOH50] = tfdata(PID.Discrete.ZOH50, 'v');
















