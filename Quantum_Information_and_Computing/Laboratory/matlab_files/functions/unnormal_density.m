function rho = unnormal_density(t)
% DENSITY_MATRIX computes an unnormalized density matrix based on 16 
% parameters stored in the input vector t

T = [t(1),           0,              0,             0;
     t(5)+1i*t(6),   t(2),           0,             0;
     t(11)+1i*t(12), t(7)+1i*t(8),   t(3),          0;
     t(15)+1i*t(16), t(13)+1i*t(14), t(9)+1i*t(10), t(4)];

rho = T'*T;
end