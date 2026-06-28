clear
%% Exercise 1: Temporal Evolution

syms C S gam delOm h real
Omega = (gam^2 + delOm^2)^(1/2);
%{
% Define the Hamiltonian
H = h*[delOm, gam; gam, -delOm];


% 1) Eigenvalues and Eigenvectors
[evec,eval] = eig(H);
disp("Eigenvalues of the Hamiltonian:")
disp(eval([1,4]))

disp("Eigenvectors")
disp(evec)
%}

% 2) Energy Gap
%{
hbar = 6.62607015e-34/(2*pi);   % [J s]

gap = zeros(100,100);
for omega = 0:10e7:10e9
    for gamma = 0:10e7:10e9
        gap((omega/10e7)+1, (gamma/10e7)+1) = 2*hbar*sqrt(omega^2+gamma^2);
    end
end

f1 = figure(1)
surf(gap*10e24)
grid on
xlabel("\Delta \omega [10^7 Hz]")
ylabel("\gamma [10^7 Hz]")
zlabel("\lambda_1-\lambda_0 [10^{-24} J]")

f2 = figure(2)
plot(0:1:100, gap(:,50)*10e24, "LineWidth",1.5)
grid on
xlabel("\Delta \omega [10^7 Hz]")
ylabel("\lambda_1-\lambda_0 [10^{-24} J]")
%}


% 3) Temporal Evolution
% |psi+/-(t)> = a0/1 |0> + b0/1 |1>
a0 = ((cos(Omega) + sin(Omega)*gam/Omega) - 1i*sin(Omega)*delOm/Omega)/sqrt(2);
b0 = ((-sin(Omega)*delOm/Omega + 1i*(cos(Omega)-gam*sin(Omega)/Omega)))/sqrt(2);

a1 = ((cos(Omega) - sin(Omega)*gam/Omega) - 1i*sin(Omega)*delOm/Omega)/sqrt(2);
b1 = ((sin(Omega)*delOm/Omega - 1i*(cos(Omega)+gam*sin(Omega)/Omega)))/sqrt(2);


% 4) Bloch Vecotrs
% x = 2 Re(a* b)
% y = 2 Im(a* b)
% z = |a|^2 -|b|^2

prod0 = simplify(expand(conj(a0)*b0));
prod1 = simplify(expand(conj(a1)*b1));

r0(1) = 2*real(prod0);
r0(2) = 2*imag(prod0);
r0(3) = simplify(expand( (real(a0)^2+imag(a0)^2)-(real(b0)^2+imag(b0)^2) ));

r1(1) = 2*real(prod1);
r1(2) = 2*imag(prod1);
r1(3) = simplify(expand( (real(a1)^2+imag(a1)^2)-(real(b1)^2+imag(b1)^2) ));

dot = simplify(expand(r0*r1'));

