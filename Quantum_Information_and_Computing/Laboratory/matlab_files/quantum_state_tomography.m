% Script to perform a quantum state tomography based on coincidence count
% data. The data is assumed to be stored in a file called "tomography.mat"
% in a subdirectory "data" to the root directory of this file. It invokes a
% set of custom functions, that are assumed to be stored in a subdirectory
% called "functions".
clear
addpath('functions')

%% Load Data from the Experiment
load("data\tomography.mat")

% Extrapolate the missing data from coinc_minus to plus and decoherence
scale_plus = sum(coinc_plus(1:4))/sum(coinc_minus(1:4));
scale_deco = sum(coinc_deco(1:4))/sum(coinc_minus(1:4));

coinc_plus([5:9, 11:15]) = coinc_minus([5:9, 11:15])*scale_plus;
coinc_deco([5:9, 11:15]) = coinc_minus([5:9, 11:15])*scale_deco;


%% Define the measurements
% Normalized basis vectors
H = [1;0];
V = [0;1];
D = 1/sqrt(2)*(H+V);
R = 1/sqrt(2)*(H+1i*V);
L = 1/sqrt(2)*(H-1i*V);

% States to be measured
Phi_m = 1/sqrt(2)*(kron(H,H) - kron(V,V));
Phi_p = 1/sqrt(2)*(kron(H,H) + kron(V,V));

% Outcome pairs in correct order
Psi = {{H,H}; {H,V}; {V,V}; {V,H}; {R,H}; {R,V}; {D,V}; {D,H};
        {D,R}; {D,D}; {R,D}; {H,D}; {V,D}; {V,L}; {H,L}; {R,L}};

for nu=1:16
    Psi{nu} = kron(Psi{nu}{1}, Psi{nu}{2});
end


%% Compute the densities for |Psi+>, |Psi-> and the Decoherent State
% By linear inversion
% Define the Pauli matrices
sigma_0 = eye(2,2);
sigma_1 = [0, 1; 1, 0];
sigma_2 = [0, -1i; 1i, 0];
sigma_3 = [1, 0; 0, -1];
pauli_matrices = {sigma_0, sigma_1, sigma_2, sigma_3};

% Define the Gamma matrices
Gamma = cell(16,1);
mu = 1;
for i=1:4
    for j=1:4
        Gamma{mu} = kron(pauli_matrices{i}, pauli_matrices{j});
        mu = mu+1;
    end
end

% Compute the density
rho_minus_linv = density_from_linear_inversion(coinc_minus, Psi, Gamma);
rho_plus_linv  = density_from_linear_inversion(coinc_plus, Psi, Gamma);
rho_deco_linv  = density_from_linear_inversion(coinc_deco, Psi, Gamma);


% By maximum likelihood
rho_minus_ml = density_from_maximum_likelihood(coinc_minus, Psi);
rho_plus_ml  = density_from_maximum_likelihood(coinc_plus, Psi);
rho_deco_ml  = density_from_maximum_likelihood(coinc_deco, Psi);


%% Error Analysis
% For linear inversion
errors_minus_linv = error_analysis(rho_minus_linv, Phi_m);
errors_plus_linv  = error_analysis(rho_plus_linv, Phi_p);
errors_deco_linv  = error_analysis(rho_deco_linv, Phi_m);

% For maximum likelihood
errors_minus_ml = error_analysis(rho_minus_ml, Phi_m);
errors_plus_ml  = error_analysis(rho_plus_ml, Phi_p);
errors_deco_ml  = error_analysis(rho_deco_ml, Phi_m);



%% Statistical Errors by Simulation
% Preallocate
errors_minus_linv_s = zeros(100,3);
errors_minus_ml_s = zeros(100,3);
errors_plus_linv_s = zeros(100,3);
errors_plus_ml_s = zeros(100,3);
errors_deco_linv_s = zeros(100,3);
errors_deco_ml_s = zeros(100,3);

for i=1:100
    % Synthetic measurements
    meas_minus = poissrnd(coinc_minus);
    meas_plus = poissrnd(coinc_plus);
    meas_deco = poissrnd(coinc_deco);

    % Get linear inversion rho
    rho_minus_linv_s = density_from_linear_inversion(meas_minus,Psi,Gamma);
    rho_plus_linv_s = density_from_linear_inversion(meas_plus,Psi,Gamma);
    rho_deco_linv_s = density_from_linear_inversion(meas_deco,Psi,Gamma);

    % Get max likelihood rho
    rho_minus_ml_sim = density_from_maximum_likelihood(meas_minus,Psi);
    rho_plus_ml_sim = density_from_maximum_likelihood(meas_plus,Psi);
    rho_deco_ml_sim = density_from_maximum_likelihood(meas_deco,Psi);

    % Errors (1 specifies that the Von-Neuman entropy is not calculated)
    errors_minus_linv_s(i,:) = error_analysis(rho_minus_linv_s,Phi_m);
    errors_minus_ml_s(i,:) = error_analysis(rho_minus_ml_sim,Phi_m);
    errors_plus_linv_s(i,:) = error_analysis(rho_plus_linv_s,Phi_p);
    errors_plus_ml_s(i,:) = error_analysis(rho_plus_ml_sim,Phi_p);
    errors_deco_linv_s(i,:) = error_analysis(rho_deco_linv_s,Phi_m);
    errors_deco_ml_s(i,:) = error_analysis(rho_deco_ml_sim,Phi_m);
end

% Mean and standard deviation of the generated samples
errors_minus_linv_mean = mean(errors_minus_linv_s);
errors_minus_ml_mean = mean(errors_minus_ml_s);
errors_plus_linv_mean = mean(errors_plus_linv_s);
errors_plus_ml_mean = mean(errors_plus_ml_s);
errors_deco_linv_mean = mean(errors_deco_linv_s);
errors_deco_ml_mean = mean(errors_deco_ml_s);

errors_minus_linv_std = sqrt(var(errors_minus_linv_s));
errors_minus_ml_std = sqrt(var(errors_minus_ml_s));
errors_plus_linv_std = sqrt(var(errors_plus_linv_s));
errors_plus_ml_std = sqrt(var(errors_plus_ml_s));
errors_deco_linv_std = sqrt(var(errors_deco_linv_s));
errors_deco_ml_std = sqrt(var(errors_deco_ml_s));