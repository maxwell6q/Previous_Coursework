% Script solely used to compute the true posterior mean for the MCMC 
% homework. It is only to be run after or by the main.m script.

% Span three grid of points to evaluate the posterior mean
% one for each parameter to be fixed
alpha_vals = 0.01:0.01:1;
beta_vals  = 0.01:0.01:1;
gamma_vals = 0.01:0.01:1;

log_post_ab = zeros(length(alpha_vals)/2, length(beta_vals));
log_post_ag = zeros(length(alpha_vals)/5, length(gamma_vals));
log_post_bg = zeros(length(beta_vals), length(gamma_vals));

% gamma fixed
for a = 1:length(alpha_vals)/2
    for b = 1:length(beta_vals)
        x = [alpha_vals(a); beta_vals(b); gamma_true];
        [~, Y_] = ode45(@(t,Y)seir_model(t,Y,x), tspan, y0);
        E = Y_(:,2);
        log_post_ab(a,b) = calc_log_posterior(x,y,E,k);
    end
end

% beta fixed
for a = 1:length(alpha_vals)/5
    for g = 1:length(gamma_vals)
        x = [alpha_vals(a); beta_true; gamma_vals(g)];
        [~, Y_] = ode45(@(t,Y)seir_model(t,Y,x), tspan, y0);
        E = Y_(:,2);
        log_post_ag(a,g) = calc_log_posterior(x,y,E,k);
    end
end

% alpha fixed
for b = 1:length(beta_vals)
    for g = 1:length(gamma_vals)
        x = [alpha_true; beta_vals(b); gamma_vals(g)];
        [~, Y_] = ode45(@(t,Y)seir_model(t,Y,x), tspan, y0);
        E = Y_(:,2);
        log_post_bg(b,g) = calc_log_posterior(x,y,E,k);
    end
end