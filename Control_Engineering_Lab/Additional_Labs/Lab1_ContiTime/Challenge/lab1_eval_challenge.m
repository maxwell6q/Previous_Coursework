% This script is used to evaluate the step response for one setpoint,
% based on the simulation of the Quanser SRV-02 + NI DAQ. 


% Name of the output variable
sOut = ScopeData;

% Retrieving the Information of the step response
sInfo = stepinfo(sOut.signals(1).values, sOut.time, ...
                 sIn.position, 'SettlingTimeThreshold', 0.05);

% Output of the relevant transient parameters
disp("Refference: 70 deg")
fprintf("Overshoot: Mp=%2.2f; RiseTime: tr=%.4f\n", ...
       sInfo.Overshoot, sInfo.RiseTime)
