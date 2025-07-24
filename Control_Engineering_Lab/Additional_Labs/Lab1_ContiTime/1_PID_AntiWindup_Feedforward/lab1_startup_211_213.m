% Matlab script to prepare the Simulink model "lab1_model_211_213.slx" of 
% the Quanser SRV-02 + NI DAQ with a PID-controller. The controller has the
% same parameters as in Lab0, derived with the Bode-Method and is augmented
% with an Anti-Windup and Feed-Forward mechanism to improve performance. In
% the section "User Input" parameters for two different validation routines
% may be chosen.
clear



%% Load Predefined Parameters
load_params_inertial_case



%% User Inputs

% Motor Parameters
% Nominal Parameters (estimated from Blackbox)
%%mld.Beq = 1.2224e-6;    % [Nm/(rad/sec)] 
%%mld.tausf = 0.0056;     % [Nm]
%%mld.Jeq = mld.Jeq;      % [kg m^2]

% Actual Parameters (estimated from Motor 1)
mld.Beq = 1.3672e-6;     % [Nm/(rad/s)]
mld.tausf = 0.0148;      % [Nm]
mld.Jeq = 6.4640e-07;    % [kg m^2]


% Validation Anti-Windup
% magnitude of reference step(s) in [deg] (may be an array of values)
windup.mag = 360;
% time for which each refference is applied [s]
windup.time = 5;


% Validation Feedforward
% stepsize of acceleration of the load [rpm/s]
feedforward.acc = 900;
% time for which acceleration is applied [s]
feedforward.time = 0.5;
% number of cycles (pos. -> 0 -> neg. -> neg. -> 0 -> pos.)
feedforward.num = 1;


% Choice of Input => "Anti-Windup"->1 or "Feedforward"->2
sIn.program = 2;


%% PID Parameters

% reduced plant transfer function
plant.km = (drv.dcgain*mot.Kt)/((mot.Req*mld.Beq) + (mot.Kt*mot.Ke));
plant.Tm = (mot.Req*mld.Jeq)/((mot.Req*mld.Beq) + (mot.Kt*mot.Ke));
plant.Ps = tf(plant.km, [(gbox.N*plant.Tm) gbox.N 0]); 

% resulting gains from bode method
%PID = computePIDGains(8, 0.15, 0.1, plant.Ps, "PID");
PID.Kp = 7.845;
PID.Ki = 100.8347;
PID.Kd = 0.0763;
PID.Tl = 9.7252e-04;

% Anit Windup
PID.t_s5 = 0.15; % 5% settling time from lab0
PID.Kw = 1/(PID.t_s5/4.5); % anit windup gain: 1/Tw, Tw=t_s5/5




%% Filter Parameters for Friction Estimation
% Butterworth high-pass for differentiating measured position
filt.butt.omega_c = 2*pi*10;
filt.butt.delta =1/sqrt(2);
filt.butt.num = [filt.butt.omega_c^2, 0];
filt.butt.den = [1, 2*filt.butt.delta*filt.butt.omega_c, filt.butt.omega_c^2];

% Low-pass noise reduction on current
filt.low.omega_ci = 2*pi*20;
filt.low.delta_i = 1/sqrt(2);
filt.low.num = filt.low.omega_ci^2;
filt.low.den = [1, 2*filt.low.delta_i*filt.low.omega_ci, filt.low.omega_ci^2];



%% Simulation Parameters from User Input
% Anti-Windup Validation (360° reference step)
windup.num = length(windup.mag)*2-1; %number of (steps + breaks)
windup.ref = zeros(1,windup.num);
windup.ref(1:2:end) = windup.mag;    %zeros between setpoints


% Feedforward Validation (acceleration profile)
feedforward.ref = ones(6*feedforward.num,1);
feedforward.ref(2:3:end) = 0;
feedforward.ref(union(3:6:end, 4:6:end)) = -1;
feedforward.ref = feedforward.ref*feedforward.acc;


% Total Simulation time
switch sIn.program
    case 1  %"Anti-Windup"
        sIn.time = windup.time * windup.num;

    case 2  %"Feedforward"
        sIn.time = feedforward.time*6 *feedforward.num;
end


%% Console Output
% Friction Parameters
if mld.Beq == 1.2224e-6
    disp("Friction from Blackbox")
else
    disp("Friction from Real Motor")
end
% Time steps to save for realtimesimulation
disp(sIn.time*1000 + 1)
