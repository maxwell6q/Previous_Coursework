function rho = density_from_linear_inversion(coinc,Psi,Gamma)
% DENSITY_FROM_LINEAR_INVERSION compues the density matrix of a 2 qubit 
% state by linear inversion. The resulting density matrix is not guaranteed
% to be physical.
%
% coinc - measured coincidences as a vector
% Psi   - the measured states as a cell array of vectors
% Gamma - the 16 tensor products of the pauli matrices as a cellarray

% Compute the B matrix
B = zeros(16,16);
for nu=1:16
    for mu=1:16
        B(nu,mu) = Psi{nu}' *Gamma{mu}* Psi{nu};
    end
end

% Compute the r vector
r = B\coinc;

% Compute the unnormalized density
rho = zeros(4, 4);  % 2-qubit system -> 4x4 matrix
for mu = 1:16
    rho = rho + r(mu)*Gamma{mu};
end

% Normalize the density
rho = rho/trace(rho);
end

