function dydt = seir_model(t, Y, x)
%SEIR_MODEL Function to describe the dynamics of the SEIR epidemic model
%
% INPUTS:
% t - time of the simulation (not used as it is a time invariant model,
%     required for ode45 solver)
% Y - current division of population in compartments
% x - parameters for the model to be used
%
% OTUPUT:
% dydt - the current change of state of the system

    S = Y(1);   % number of susceptible
    E = Y(2);   % number of exposed
    I = Y(3);   % number of infectious
    R = Y(4);

    alpha = x(1);   % incubation rate in 1/day
    beta  = x(2);   % effective contact rate in 1/day/person
    gamma = x(3);   % recovery rate in 1/day

    % Model dynamics
    dS = -beta*S*I;
    dE =  beta*S*I - alpha*E;
    dI =  alpha*E  - gamma*I;
    dR =  gamma*I;

    dydt = [dS; dE; dI; dR];
end