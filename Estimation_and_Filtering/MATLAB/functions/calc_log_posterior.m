function log_pi_X = calc_log_posterior(x,y,E,k)
% CALC_LOG_POSTERIOR returns the value of the log-posterior p(X|y) up to a 
% constant offset. 
%
% INPUTS:
% x - point at which the posterior shall be evaluated
% y - measurements vector
% E - predicted cumulative number of exposed population using X
% k - shape factor from noise model
%
% OUTPUT:
% pog_pi_X - the log posterior log(p(X=x|y))

    alpha = x(1);       % current parameter
    lambda = alpha*E;   % current estimate of daily new infections
    priorMean = [0.2;0.4;0.4];   % mean of prior

    % log posterior up to a constant offset
    likelihood_part = -(k*y)./(lambda);
    prior_part = -0.5*(x-priorMean)'*(x-priorMean);
    log_pi_X = sum(likelihood_part) + prior_part;
end
