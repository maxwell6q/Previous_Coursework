% This script is used to evaluate the step response for one setpoint,
% based on the simulation of the Quanser SRV-02 + NI DAQ. 


% Name of the output variable
sOut = ScopeData;

% Retrieving the Information of the step response
sInfo = stepinfo(sOut.signals(1).values, sOut.time, ...
                 60, 'SettlingTimeThreshold', 0.05);

% Output of the relevant transient parameters
disp("Refference: 60 deg")
fprintf("Overshoot: Mp=%2.2f; SettlingTime: ts=%.4f; SamplingTime: Ts=%.4f\n", ...
       sInfo.Overshoot, sInfo.SettlingTime, sIn.T_s)