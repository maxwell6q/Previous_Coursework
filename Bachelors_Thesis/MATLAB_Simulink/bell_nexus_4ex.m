clear
%% Flight Simulation of the Bell Nexus 4EX around two Trim Conditions
% as system parameters, a choice can be made for one of two sets of PID 
% controllers and if different dynamics are taken into account.
% simulation parameters include the form of the disturbance and setpoints
% for controlled variables 
% longitudinal state vector: x=[u w q theta]', input: u=del_e
% with u[m/s], w[m/s], q[rad/s], theta[rad], del_e[rad]
% lateral state vector: x=[v p r phi]', input: u=[del_r del_a]'
% with v[m/s], p[rad/s], r[rad/s], phi[rad], del_r[rad], del_a[rad]

%   Ver.      Date        Author
%   1.0    24.07.2024    M. Pries
%--------------------------------------



%% Load Parameters from External Script
bell_nexus_4ex_params



%% System Parameters
% trim condition (1=150mph, 2=120mph)
tc = 1;

% set of controller parameters (1=new, 2=literature)
con_param = 1;

% include actuator? (1=y, 2=n(comp. to literature))
act = 1;

% include pid derivative lag? (1=y, 2=n(comp. to literature))
dl = 1;

% include pre-filter? (1=y, 2=n(comp. to literature))
pf = 2;

% display poles of con. system? (1=y, 2=n) 
p = 2;

% design new controller? (1=y, 2=n (based on chosen controller and sys))
% for elevator channel
cd_elev = 2;
% for aileron channel
cd_ailer = 2;



%% Simulation Parameters
% run simulation? (1=y, 2=n (or only load params for analysis/simulink))
simu = 2;

% simulation time and step time [s]
simt = 30;
st = 0.01;

% wind input (1=step, 2=random sinusoid)
wc = 2;

% wind step input; time [s] and magnitude [m/s] ([uw, vw, ww])
w_st_time = 4;
wind_lon = [0 0 0];
wind_lat = [0 0 0];

% pilot command input; start time, magnitude & slope (same sign!; 0 = step)
% will be ignored if act=pf=dl=2 -> (non-causal tf from command to input)
theta_st_time = 4;      % [s]
theta_sp = 0;           % [deg]
theta_slope = 0;        % [deg/s]

phi_st_time = 4;        % [s]
phi_sp = 0;             % [deg]
phi_slope = 0;          % [deg/s]

r_st_time = 4;          % [s]
r_sp = 0;               % [deg/s]
r_slope = 0;            % [deg/s^2]



%% Parameter Display
% system params
disp(trim_disp{tc})
disp(sys_disp{act+dl-1,act})   % reduced or full system
disp(pf_disp{pf})              % include pre-filter or not
disp(con_disp{con_param})      % which controller set is used
disp('---------------------------')

% simulation params
sp = {'',theta_sp, phi_sp, r_sp};
w_disp = {wind_lon, '', wind_lat, ''};

% display wind conditions -> if step, magnitude is shown
disp(wind_disp{1,wc})
disp([wind_disp{wc+1,1}, num2str(w_disp{wc})])
disp([wind_disp{wc+1,2}, num2str(w_disp{wc+2})])
disp('---------------------------')

% display setpoints -> if nonzero, magnitude is shown
disp(['Pitch Angle' sp_disp{(theta_sp~=0)+1} num2str(sp{(theta_sp~=0)+1})])
disp(['Bank Angle ' sp_disp{(phi_sp~=0)+1} num2str(sp{(phi_sp~=0)*2+1})])
disp(['Yaw Rate   ' sp_disp{(r_sp~=0)+1} num2str(sp{(r_sp~=0)*3+1})])



%% Data Extraction
[A_lon, B_lon, A_lat, B_lat, u0] = tc_{tc,:};
[kp_elevator, ki_elevator, kd_elevator] = elevator_{con_param,:};
[kp_aileron, ki_aileron, kd_aileron] = aileron_{con_param,:};



%% Wind Disturbance Augmentation
Bw_lon = A_lon*Tw_lon;
Bw_lat = A_lat*Tw_lat;

% matrix augmentation
B_lon_ext = [B_lon, Bw_lon];
C_lon_ext = [C_lon; eye(2,4)];
D_lon_ext = blkdiag(D_lon, Tw_lon(1:2,:)); %blockdiagnoal matrix

B_lat_ext = [B_lat, Bw_lat];
C_lat_ext = [C_lat; eye(1,4)];
D_lat_ext = blkdiag(D_lat, Tw_lat(1,:));



%% Extended State Space Model of the 4EX
% longitudinal dynamics
sys_lon = ss(A_lon, B_lon_ext, C_lon_ext, D_lon_ext);
sys_lon.InputName = {'del_e'; 'uw'; 'vw'; 'ww'};                               
sys_lon.OutputName = {'u_gr'; 'w_gr'; 'q'; 'theta';'u_air';'w_air'};

% lateral dynamics
sys_lat = ss(A_lat, B_lat_ext, C_lat_ext, D_lat_ext);
sys_lat.InputName = {'del_r'; 'del_a'; 'uw'; 'vw'; 'ww'};
sys_lat.OutputName = {'v_gr'; 'p'; 'r'; 'phi'; 'v_air'};



%% Controller Models
% param for include derivative lag check
% elevator -> pitch angle (theta) feedback
elev_pid_param = {kp_elevator, ki_elevator, kd_elevator, deriv_lag};
elev_con = pid(elev_pid_param{1:5-dl});
elev_con.InputName = 'e_theta';
elev_con.OutputName = 'del_e_com';

% rudder -> yaw rate (r) feedback
rudder_pid_param = {kp_rudder, ki_rudder, kd_rudder, deriv_lag};
rudder_con = pid(rudder_pid_param{1:5-dl});
rudder_con.InputName = 'e_r';
rudder_con.OutputName = 'del_r_com';

% aileron -> bank angle (phi) feedback
ailer_pid_param = {kp_aileron, ki_aileron, kd_aileron, deriv_lag};
ailer_con = pid(ailer_pid_param{1:5-dl});
ailer_con.InputName = 'e_phi';
ailer_con.OutputName = 'del_a_com';

% deriv_lag = 0 not possible in simulink -> small for min effect
if dl == 2
    deriv_lag = 0.005; 
end



%% Prefilter Models
% include prefilter check => if no prefilter: static gain 1
den_pf = {[elev_pf_tau,1]; 1};

% elevator
elev_pf = tf(1,den_pf{pf});
elev_pf.InputName = 'theta_com';
elev_pf.OutputName = 'theta_com_f';



%% Actuator Models
% include actuator check => if no actuator: static gain 1
den_act = {[tau,1]; 1};

% elevator
elev_act = tf(1,den_act{act});
elev_act.InputName = 'del_e_com';
elev_act.OutputName = 'del_e';

% rudder
rudder_act = tf(1,den_act{act});
rudder_act.InputName = 'del_r_com';
rudder_act.OutputName = 'del_r';

% aileron
ailer_act = tf(1,den_act{act});
ailer_act.InputName = 'del_a_com';
ailer_act.OutputName = 'del_a';



%% Error Calculation Blocks (negative feedback)
if (act+dl) ~= 4   % check to prevent non-causality
    % elevator (pitch angle)
    elev_s = sumblk('e_theta = theta_com_f - theta');
    
    % rudder (yaw rate)
    rudder_s = sumblk('e_r = r_com - r');
    
    % aileron (bank angle)
    ailer_s = sumblk('e_phi = phi_com - phi');
else
    % only static gain of -1 -> set point = 0
    elev_s = tf(-1);
    elev_s.InputName = 'theta';
    elev_s.OutputName = 'e_theta';

    rudder_s = tf(-1);
    rudder_s.InputName = 'r';
    rudder_s.OutputName = 'e_r';

    ailer_s = tf(-1);
    ailer_s.InputName = 'phi';
    ailer_s.OutputName = 'e_phi';
end



%% System Connection
% suppress warning for unused inputs/outputs
warning_id_in = 'Control:combination:connect9';
warning_id_out = 'Control:combination:connect10';
warning('off', warning_id_in);
warning('off', warning_id_out);

% longitudinal dynamics
inputs_lon = {'theta_com'; 'uw'; 'vw'; 'ww'};
if (act+dl) == 4   % check to prevent non-causality
    inputs_lon = inputs_lon(2:end);
end
outputs_lon = {'theta'; 'del_e'; 'u_gr'; 'w_gr'; 'u_air'; 'w_air'};
con_sys_lon = connect(sys_lon, elev_con, elev_act, elev_pf, elev_s, ...
              inputs_lon, outputs_lon);

% lateral dynamics
inputs_lat = {'r_com'; 'phi_com'; 'uw'; 'vw'; 'ww'};
if (act+dl) == 4   % check to prevent non-causality
    inputs_lat = inputs_lat(3:end);
end
outputs_lat = {'r'; 'phi'; 'del_r'; 'del_a'; 'v_gr'; 'v_air'};
con_sys_lat = connect(sys_lat, rudder_con, rudder_act, rudder_s,...
              ailer_con, ailer_act, ailer_s, inputs_lat, outputs_lat);


%% Simulation Inputs
% inputs are arrays with the input values for each time step
% => length = simulation time [s] / step time [s] +1
% time
t = 0:st:simt;

% zeros column vector
zeros_vec = zeros(length(t),1);

% wind
% one dimensional unit step ([0 ... 0 , 1 ... 1]'; last 0 at t=st_time-st)
wind_st = zeros_vec;
wind_st(t >= w_st_time) = 1;

% wind step input, three dimensional [m/s]
wind_step_lon = wind_st.*wind_lon;
wind_step_lat = wind_st.*wind_lat;

% wind sinusoid input 
wind_v = zeros(length(t),3);
wind_u = zeros(length(t),3);
wind_w = zeros(length(t),3);
for i=1:3 %matrix for each direction, with modes in columns
    wind_u(:,i) = a_u(i)*cos(t*omega(i) + rand()*2*pi);
    wind_v(:,i) = a_v(i)*cos(t*omega(i) + rand()*2*pi);
    wind_w(:,i) = a_w(i)*cos(t*omega(i) + rand()*2*pi);
end
% column vectors for different directions
wind_sine = [sum(wind_u,2) sum(wind_v,2) sum(wind_w,2)];


% pilot inputs
% ramps -> linear functions with zero at t=st_time (nonzero before)
theta_ramp = (t' - theta_st_time)*theta_slope;
r_ramp = (t' - r_st_time)*r_slope;
phi_ramp = (t' - phi_st_time)*phi_slope;

% choice -> step or ramp (nonzero values before t=st_time are omitted)
theta_choice = {theta_sp, theta_ramp(t>=theta_st_time)};
r_choice = {r_sp, r_ramp(t>=r_st_time)};
phi_choice = {phi_sp, phi_ramp(t>=phi_st_time)};

% selecting theta command [rad]
theta_command = zeros_vec;
theta_command(t>=theta_st_time) = theta_choice{(theta_slope~=0)+1}*pi/180;
theta_command = sat(theta_command, theta_sp*pi/180); %saturation of ramp

% selecting r command [rad/s]
r_command = zeros_vec;
r_command(t>=r_st_time) = r_choice{(r_slope~=0)+1}*pi/180;
r_command = sat(r_command, r_sp*pi/180); %saturation of ramp

% selecting phi command [rad]
phi_command = zeros_vec;
phi_command(t>=phi_st_time) = phi_choice{(phi_slope~=0)+1}*pi/180;
phi_command = sat(phi_command, phi_sp*pi/180); %saturation of ramp



% check if wind as step or sinusoid
if wc==1    %wind as step input
    inp_lon = [theta_command, wind_step_lon];
    inp_lat = [r_command, phi_command, wind_step_lat];
else        %wind as sinusoid
    inp_lon = [theta_command, wind_sine];
    inp_lat = [r_command, phi_command, wind_sine];
end

% check to prevent non-causality
if (act+dl) == 4
    inp_lon = inp_lon(:,2:end);
    inp_lat = inp_lat(:,3:end);
end



%% Simulation and Results
if simu == 1
    %% Simulation
    y_lon = lsim(con_sys_lon, inp_lon, t);
    y_lat = lsim(con_sys_lat, inp_lat, t);
    
    
    
    %% Data Extraction and Unit Transformation [rad]->[deg]
    % longitudinal dynamics 
    theta_d = y_lon(:,1)*(180/pi);
    del_e_d = y_lon(:,2)*(180/pi);
    
    u_gr = y_lon(:,3);
    u_air = y_lon(:,5);
    
    w_gr = y_lon(:,4);
    w_air = y_lon(:,6);
    
    
    % lateral dynamics
    r_d = y_lat(:,1)*(180/pi);
    del_r_d = y_lat(:,3)*(180/pi);
    
    phi_d = y_lat(:,2)*(180/pi);
    del_a_d = y_lat(:,4)*(180/pi);
    
    v_gr = y_lat(:,5);
    v_air = y_lat(:,6);
    
    
    
    %% Angle of Attack Deviation [deg]
    del_alpha = atan2(w_air, u_air+u0)*180/pi;
    
    

    %% Plot
    % plots will only be generated if any command/disturbance is given
    % to save a plot: saveas(gcf, 'figs/my_fig.png');
    % longitudinal dynamics
    if any(inp_lon(:)>0)
        % pitch angle and elevator deflection
        figure(1)
        yyaxis left
        plot(t,theta_d)
        ylim([-max(abs(theta_d))*1.1 max(abs(theta_d))*1.1])
        ylabel('Pitch Angle \theta [deg]')
        
        yyaxis right
        plot(t,del_e_d)
        ylim([-max(abs(del_e_d))*1.1 max(abs(del_e_d))*1.1])
        ylabel('Elevator Deflection \delta_e [deg]')
        
        legend('\theta', '\delta_e')
        grid on
        xlabel('Time [s]')
        
        
        % longitudinal speeds: u_gr, u_air, difference
        figure(2)
        hold on
        plot(t, u_gr)
        plot(t, u_air)
        plot(t, u_air-u_gr)
        ylim([-max(abs([u_air; u_gr]))*1.1 max(abs([u_air; u_gr]))*1.1])
        grid on
        ylabel('Longitudinal Speed Deviation [m/s]')
        xlabel('Time [s]')
        legend('u_{ground}', 'u_{air}', '\Delta u_{Air-Ground}')
        
        
        % vertical speeds: w_air, w_gr, difference
        figure(3)
        hold on
        plot(t, w_gr)
        plot(t, w_air)
        plot(t, w_air-w_gr)
        ylim([-max(abs([w_air; w_gr]))*1.1 max(abs([w_air; w_gr]))*1.1])
        grid on
        ylabel('Vertical Speed [m/s]')
        xlabel('Time [s]')
        legend('w_{ground}', 'w_{air}', '\Delta w_{Air-Ground}')
        
        
        %Angle of Attack
        figure(4)
        plot(t,del_alpha)
        ylim([-max(abs(del_alpha))*1.1 max(abs(del_alpha))*1.1])
        grid on
        ylabel('Deviation of Angle of Attack \Delta\alpha [deg]')
        xlabel('Time [s]')
    end
    
    % lateral dynamics
    if any(inp_lat(:)>0)
        % yaw rate and rudder deflection
        figure(5)
        yyaxis left
        plot(t,r_d)
        ylim([-max(abs(r_d))*1.1 max(abs(r_d))*1.1])
        ylabel('Yaw Rate r [deg/s]')
        
        yyaxis right
        plot(t,del_r_d)
        ylim([-max(abs(del_r_d))*1.1 max(abs(del_r_d))*1.1])
        ylabel('Rudder Deflection \delta_r [deg]')
        
        legend('r', '\delta_r')
        grid on
        xlabel('Time [s]')
        
        
        % bank angle and aileron deflection
        figure(6)
        yyaxis left
        plot(t,phi_d)
        ylim([-max(abs(phi_d))*1.1 max(abs(phi_d))*1.1])
        ylabel('Bank Angle \phi [deg]')
        
        yyaxis right
        plot(t, del_a_d)
        ylim([-max(abs(del_a_d))*1.1 max(abs(del_a_d))*1.1])
        ylabel('Aileron Deflection \delta_a [deg]')
        
        legend('\phi', '\delta_a')
        grid on
        xlabel('Time [s]')
        
        
        % lateral speeds: v_air, v_gr, difference
        figure(7)
        hold on
        plot(t, v_gr)
        plot(t, v_air)
        plot(t, v_air-v_gr)
        ylim([-max(abs([v_air; v_gr]))*1.1 max(abs([v_air; v_gr]))*1.1])
        grid on
        ylabel('Latteral Speed [m/s]')
        xlabel('Time [s]')
        legend('v_{ground}', 'v_{air}', '\Delta v_{Air-Ground}')
    end
end



%% Pole Display
if p == 1
    disp('Longitudinal System:')
    damp(con_sys_lon)
    disp(' ')
    disp('Lateral System:')
    damp(con_sys_lat)
end



%% Controller Design
% design based on the system with actuator and the chosen controller as
% starting point => tf from set point to controlled variable
% poles in compensator represent integ. and deriv. lag => only move zeros
if cd_elev == 1
    % longitudinal dynamics
    inp_lon_ol = {'del_e_com'; 'uw'; 'vw'; 'ww'};
    sys_lon_ol = connect(sys_lon, elev_act, inp_lon_ol, outputs_lon);
    G_lon = tf(sys_lon_ol('theta', 'del_e_com')); 
    controlSystemDesigner('rlocus', G_lon, elev_con)
end
if cd_ailer == 1
    % lateral dynamics
    inp_lat_ol = {'del_r_com';'del_a_com'; 'uw'; 'vw'; 'ww'};
    sys_lat_ol = connect(sys_lat, rudder_act, rudder_con, rudder_s, ...
                 ailer_act, inp_lat_ol, outputs_lat);
    G_lat = tf(sys_lat_ol('phi', 'del_a_com'));
    controlSystemDesigner('rlocus', G_lat, ailer_con)
end



%% Function Definition for Saturation
function com_sat = sat(com, sp)
%function to apply a saturation to a ramp function 
%com is the ramp, sp is the saturation value
    if sign(sp) > 0 
        com_sat = min(com, sp);
    else
        com_sat = max(com, sp);
    end
end



%% Function Definition for Finding Extreme Points and Slopes
maxWithSign = @(vec) vec(find(abs(vec) == max(abs(vec)), 1));
slope = @(vec) diff(vec)/st;