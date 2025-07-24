% This script is used to evaluate the step response for different setpoints
% based on the simulation of the Quanser SRV-02 + NI DAQ. 
% It is to be run after the simulation on the blackbox model or the real
% model.


% Name of the output variable
sOut = ScopeData;

% Port of the scope to analyze (1 for nominal; 2 for integral)
sOut.inport = 1;

% Get the info from the output
sInfo = stepinfo(sOut.signals(sOut.inport).values, ...
                   sOut.time, sIn.position, 'SettlingTimeThreshold', 0.05);
Mp  = sInfo.Overshoot;
ts5 = sInfo.SettlingTime-sIn.t1;

% Output of the transient parameters
fprintf("Overshoot: Mp=%2.2f;  SettlingTime: t_s5=%.3f\n",Mp,ts5)

