function errors = error_analysis(rho, Psi, vnm)
% ERROR_ANALYSIS computes the fidelity, concurrence and Von-Neumann entropy
% of a measured 2 qubit density matrix against the prepared state
%
% rho - measured/computed density matrix
% Psi - prepared state
% vnm - choice variable if the Von-Neumann entropy should be calculated
%       (not needed for statistical simulation)
% 
% errors - a vactor containing the error measures

% Fidelity
F = Psi' *rho* Psi;

% Concurrence
sigma_2 = [0, -1i; 1i, 0];
spin_flip = kron(sigma_2, sigma_2);
R = rho*spin_flip*rho.'*spin_flip;
eig_R = sort(real(eig(R)), 'descend');  % real part => sometimes +i*1e-16
C = max(0, sqrt(eig_R(1)) - sum(sqrt(eig_R(2:4))));

% Von-Neumann entropy (non-pysical still has real values
if ~exist('vnm','var')
    eig_rho = real(eig(rho));
    eig_rho(eig_rho<=0) = eps;
    S = -sum(eig_rho.*log2(eig_rho));

    % Output
    errors = [F,C,S];
else
    % Output
    errors = [F,C];
end

