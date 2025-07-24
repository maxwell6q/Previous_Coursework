%% Homework for the course Estimation and Filtering on Markov Chains
% This script performs an estimation of dynamic model parameters of a SEIR
% compartment model for the spread of COVID-19 in India in June 2020.
% It generates a sysnthetic set of measurements and estimates the model
% parameters using the Metropolis Hastings algorithm to realize an
% implementation of a desired Markov Chain, and Monte Carlo integration.
clear
addpath('functions')
addpath('scripts')
rng("default")      % Ensures reproducability, not used during tuning


%% User inputs
% Should plots be generated?
generate_plots = 0;     % 0:no,  1:yes

% Should the true posterior be computed?
% (takes significant time and all related info is included in the report)
comp_true_post = 0;     % 0:no,  1:yes

% Noise for measurements => determined by shape factor, small => more noise
k = 20;

% Length of the Markov Chain
N = 10000;

% Fraction of samples to discard as burn-in
burn_in = 0.25;

% Starting point for alpha and beta (as column vector)
x_0 = [0.2; 0.4; 0.4];

% Covariance of the random walk
Sigma_c = diag([1e-4, 3e-3, 5e-4]);


%% Parameter Values for the SEIR Model
N_p = 1000;             % total zize of population
alpha_true = 0.071;     % incubation rate in 1/day
beta_true  = 0.476;     % effectiv contact rate in 1/day/person
gamma_true = 0.286;     % recovery rate in 1/day
x_true = [alpha_true; beta_true; gamma_true];
E0 = 1e-3*N_p;          % number of initially exposed
tspan = 0:1:20;         % time span of simulation in days
y0 = [N_p-E0,E0,0,0];   % initial conditions on population fractions


%% Model simulation and measurement model
[t, Y] = ode45(@(t,Y)seir_model(t,Y,x_true), tspan, y0);
lambda = alpha_true*Y(:,2); % true daily new infected
y = gamrnd(k, lambda/k);          % measurements


%% MCMC
x = x_0;            % starting point for chain
X = zeros(3,N);     % initialize the chain
X(:,1) = x;         % log first value
a_count = 0;        % initialize acceptance count

% simulate the model with the initial x0 and extract E(x0)
[~,Y_x] = ode45(@(t,Y)seir_model(t,Y,x), tspan, y0);
E_x = Y_x(:,2);
log_pi_prev = calc_log_posterior(x,y,E_x,k);    % log posterior at X=x0

% start the Markov Chain
for i = 2:N
    log_pi_x = log_pi_prev;     % assign previous value
    
    c = mvnrnd(x,Sigma_c,1)';   % proposed sample
    if all(c>0)  %physical plausibility / prevent log(0) in log posterior
        % simulate the model with proposed c and get E(c)
        [~,Y_c] = ode45(@(t,Y)seir_model(t,Y,c), tspan, y0);
        E_c = Y_c(:,2);

        log_pi_c = calc_log_posterior(c,y,E_c,k);% log posterior at X=c
        a = min(0, log_pi_c-log_pi_x);           % log acceptance probab.
        u = log(rand);

        if u<a  % c is accepted
            x=c;
            a_count = a_count+1;
            log_pi_prev = log_pi_c;
        else
            log_pi_prev = log_pi_x;
        end
    end
    X(:,i) = x; % store samples
end

% Monte carlo integration
discarded  = N*burn_in;             % number of burn in samples
considered = N-discarded;           % number of considered
hat_x = mean(X(:,discarded:end),2); % estimate of MVE

% Display results
disp("Acceptance rate in percent:")
disp(a_count/N *100 +" %")
disp("")
disp("MVE considering the last " +considered+ " samples:")
disp(hat_x)
disp("True Parameters:")
disp(x_true)


%% Compute the posterior mean numerically
if comp_true_post == 1
    comp_true_post_scr
end


%% Generate the Plots
if generate_plots == 1
    generate_plots_scr
end