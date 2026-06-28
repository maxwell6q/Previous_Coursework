function loss = squared_loss(t, Psi, C)
% SQUARED_LOSS computes the squared loss of a proposed density matrix wrt
% the measurement coincidences
%
% t   - proposed parameters in denisty matrix
% Psi - measured states, as a cell array
% C   - coincidences

rho = unnormal_density(t);

loss = 0;
for nu=1:length(C)
    % real for numerical stability => might get 1i*1e-16 => minim. error
    loss = loss + real((C(nu) - Psi{nu}' *rho* Psi{nu})^2 / C(nu));
end