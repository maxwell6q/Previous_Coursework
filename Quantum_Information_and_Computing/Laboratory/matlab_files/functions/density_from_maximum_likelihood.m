function rho = density_from_maximum_likelihood(coinc, Psi)
% DENSITY_FROM_MAXIMUM_LIKELIHOOD compues the density matrix of a 2 qubit 
% state by maximum likelihood. The resulting density matrix is the one,  
% that best describes the data while beig physical.
%
% coinc - measured coincidences as a vector
% Psi   - the measured states as a cell array of vectors

% Initial guess for t
t0 = ones(16,1);

% Optimizer options
options = optimoptions('fminunc','Display','off',...
                       MaxFunctionEvaluations=1e4); % increase max evals

% Find optimal t
t_opt = fminunc(@(t)squared_loss(t,Psi,coinc), t0, options);

% Unnormalized density
rho = unnormal_density(t_opt);

% Normalized density
rho = rho/trace(rho);
end

