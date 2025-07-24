% Script solely used to generate plots for the MCMC homework. It is only to
% be run after or by the main.m script

% Plot of the measurement
f1 = figure(1);
plot(t,lambda)
hold on
plot(t,y)
hold off
grid on
xlabel("Time [d]"); ylabel("Daily new infections")
legend("True", "Measurements")


% Plot the markov chain
f2 = figure(2);
plot3(X(1,discarded:end), X(2,discarded:end), X(3,discarded:end), 'ko--')
hold on
plot3(X(1,1:discarded), X(2,1:discarded), X(3,1:discarded),'o--','Color', [0.7,0.7,0.7])
plot3(X(1,1), X(2,1), X(3,1), 'r*')
plot3(x_true(1), x_true(2), x_true(3), 'g*')
xlabel("\alpha"); ylabel("\beta"); zlabel("\gamma"); 
title("MC evolution ("+N+" iterations)");
legend("","", "Starting Point", "True Parameters");
grid on
hold off


% Plot the dynamic system with the true parameters and the estimated ones
[t,Y_est] = ode45(@(t,Y)seir_model(t,Y,hat_x), tspan, y0);
f3 = figure(3);
plot(t,Y, "-")
hold on
grid on
plot(t,Y_est,"--")
hold off
xlabel("Time [d]"); ylabel("Population in Compartment"); title("SEIR-Dynamics");
legend("True Susceptible","True Exposed","True Infected","True Recovered", ...
       "Est. Susceptible","Est. Exposed","Est. Infected","Est. Recovered");


% Traceplots of the parameters
f4 = figure(4);
subplot(1,3,1)
plot(discarded:N, X(1,discarded:end))
hold on
plot([discarded,N], hat_x(1)*[1,1], LineWidth=1.5)
hold off
grid on
xlabel("Iterations"); ylabel("\alpha");
xlim([discarded,N]);
legend("Markov-Chain", "Mean")

subplot(1,3,2)
plot(discarded:N, X(2,discarded:end))
hold on
plot([discarded,N], hat_x(2)*[1,1], LineWidth=1.5)
hold off
grid on
xlabel("Iterations"); ylabel("\beta");
xlim([discarded,N]);
legend("Markov-Chain", "Mean")

subplot(1,3,3)
plot(discarded:N, X(3,discarded:end))
hold on
plot([discarded,N], hat_x(3)*[1,1], LineWidth=1.5)
hold off
grid on
xlabel("Iterations"); ylabel("\gamma");
xlim([discarded,N]);
legend("Markov-Chain", "Mean")
sgtitle("Traceplots of the estimated parameters after burn in")


% plot the posterior
if comp_true_post == 1
    f5 = figure(5);
    subplot(1,3,1)
    posterior = log_post_ab - max(log_post_ab);
    p = pcolor(alpha_vals(1:end/2), beta_vals, exp(posterior)');
    p.FaceColor = 'interp';
    xlabel('\alpha'); ylabel('\beta');
    title("\gamma = "+gamma_true);

    subplot(1,3,2)
    posterior = log_post_ag - max(log_post_ag);
    p = pcolor(alpha_vals(1:end/5), gamma_vals, exp(posterior)');
    p.FaceColor = 'interp';
    xlabel('\alpha'); ylabel('\gamma');
    title("\beta = "+beta_true)

    subplot(1,3,3)
    posterior = log_post_bg - max(log_post_bg);
    p = pcolor(beta_vals, gamma_vals, exp(posterior)');
    p.FaceColor = 'interp';
    xlabel('\beta'); ylabel('\gamma');
    title("\alpha = "+alpha_true)

    sgtitle('True Posterior (one parameter fixed)');
end