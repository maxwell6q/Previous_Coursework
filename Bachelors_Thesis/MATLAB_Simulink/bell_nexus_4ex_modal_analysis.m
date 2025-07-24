clear
%% Modal Analysis of the Bell Nexus 4EX
% this script is designed only for the specific aircraft dynamics
% eigenvalues and eigenvectors have been identified manually, the order
% might change for other aircraft
% longitudinal state vector: x=[u w q theta]', input: u=del_e
% with u[m/s], w[m/s], q[rad/s], theta[rad], del_e[rad]
% lateral state vector: x=[v p r phi]', input: u=[del_r del_a]'
% with v[m/s], p[rad/s], r[rad/s], phi[rad], del_r[rad], del_a[rad]
% to display the results, remove the semicolons

%   Ver.      Date        Author
%   1.0    24.07.2024    M. Pries
%--------------------------------------



%% Load Parameters from External Script
bell_nexus_4ex_params



%% System Parameters and Data Extraction
% trim condition (1=150mph, 2=120mph)
tc = 1;

% data extraction
[A_lon, B_lon, A_lat, B_lat, u0] = tc_{tc,:};



%% Longitudinal Dynamics
[evec_lon, eval_lon] = eig(A_lon);
eval_lon = diag(eval_lon);

% short period mode
eval_SP = eval_lon(1);          % eigenvalue extraction
evec_SP = evec_lon(:,1:2);      % eigenvector extraction (column vectors)
n_SP = real(eval_SP);           % real part of eval
omega_SP = imag(eval_SP);       % imaginary part eval => damped frequency
damping_SP = -n_SP/abs(eval_SP);% damping coefficient
omega_0_SP = -n_SP/damping_SP;  % natural frequency

t_half_SP = log(2)/abs(n_SP);   % halving time 
T_SP = 2*pi/omega_SP;           % period of oscillation

A1_SP = sum(evec_SP,2);         % add the rows of evecs (a1+a2)
A2_SP = 1i*diff(evec_SP,1,2);   % subtract the rows of evecs (j(a1-a2)
Mag_SP = sqrt(A1_SP.^2 + A2_SP.^2);% magnitudes of mode
Mag_SP_norm = Mag_SP/Mag_SP(4); % magnitudes normalized by theta


% phugoid mode
eval_PH = eval_lon(3);
evec_PH = evec_lon(:,3:4);
n_PH = real(eval_PH);
omega_PH = imag(eval_PH);
damping_PH = -n_PH/abs(eval_PH);
omega_0_PH = -n_PH/damping_PH;

t_half_PH = log(2)/abs(n_PH);
T_PH = 2*pi/omega_PH;

A1_PH = sum(evec_PH,2);
A2_PH = 1i*diff(evec_PH,1,2);
Mag_PH = sqrt(A1_PH.^2 + A2_PH.^2);
Mag_PH_norm = Mag_PH/Mag_PH(4);


% influence of controls
sys_lon = ss(A_lon, B_lon, eye(4), zeros(4,1)); % ss representation
[msys_lon,blks_lon] = modalreal(sys_lon,Normalize=true); % modal system
B_lon_mod = msys_lon.B; % modal control matrix



%% Lateral Dynamics
[evec_lat, eval_lat] = eig(A_lat);
eval_lat = diag(eval_lat);

% roll mode
eval_roll = eval_lat(1);
evec_roll = evec_lat(:,1);

t_half_roll = log(2)/abs(eval_roll);
Mag_roll = evec_roll;
Mag_roll_norm = Mag_roll/Mag_roll(4);


% dutch roll mode
eval_dr = eval_lat(2);
evec_dr = evec_lat(:,2:3);
n_dr = real(eval_dr);
omega_dr = imag(eval_dr);
damping_dr = -n_dr/abs(eval_dr);
omega_0_dr = -n_dr/damping_dr;

t_half_dr = log(2)/abs(n_dr);
T_dr = 2*pi/omega_dr;

A1_dr = sum(evec_dr,2);
A2_dr = 1i*diff(evec_dr,1,2);
Mag_dr = sqrt(A1_dr.^2 + A2_dr.^2);
Mag_dr_norm = Mag_dr/Mag_dr(4);


% spiral mode
eval_spir = eval_lat(4);
evec_spir = evec_lat(:,4);

t_half_spir = log(2)/abs(eval_spir);
Mag_spir = evec_spir;
Mag_spir_norm = Mag_spir/Mag_spir(4);


% influence of controls
sys_lat = ss(A_lat, B_lat, eye(4), zeros(4,2));
[msys_lat,blks_lat] = modalreal(sys_lat,Normalize=true);
B_lat_mod = msys_lat.B;