function pidGains = computePIDGains(alpha, t_s, M_p, Ps, controllerType)
% computePIDGains Computes the controller gains (PID, PI, or PD) using Bode's method.
%
% Syntax:
%   pidGains = computePIDGains(alpha, t_s, M_p, Ps, controllerType)
%
% Inputs:
%   alpha         - Ratio between integral and derivative time constants (T_i / T_d)
%   t_s           - Desired settling time (seconds)
%   M_p           - Desired maximum overshoot (0 to 1)
%   Ps            - Plant transfer function (as a tf object)
%   controllerType- Type of controller ('PID', 'PI', or 'PD')
%
% Outputs:
%   pidGains - Struct containing the controller gains:
%              - Kp: Proportional gain (V/rad)
%              - Ki: Integral gain (V/rad·s) [only for 'PID' and 'PI']
%              - Kd: Derivative gain (V·s/rad) [only for 'PID' and 'PD']
%              - Tl: Time constant for the "real derivative" (Tl >= 1/(2*w_gc)) [only for 'PID' and 'PD']
%
% Example:
%   Ps = tf([1], [0.01 0.1 1]);
%   alpha = 4;
%   t_s = 0.15;
%   M_p = 0.1;
%   controllerType = 'PID';
%   pidGains = computePIDGains(B_eq, alpha, t_s, M_p, Ps, controllerType);

% Validate inputs
validateattributes(alpha, {'numeric'}, {'positive'}, mfilename, 'alpha', 2);
validateattributes(t_s, {'numeric'}, {'positive'}, mfilename, 't_s', 3);
validateattributes(M_p, {'numeric'}, {'positive', '<=', 100}, mfilename, 'M_p', 4);
validateattributes(Ps, {'tf'}, {}, mfilename, 'Ps', 5);
validateattributes(controllerType, {'char', 'string'}, {'nonempty'}, mfilename, 'controllerType', 6);

% Convert controllerType to uppercase for consistency
controllerType = upper(controllerType);

% Check if the controllerType is valid
validControllers = {'PID', 'PI', 'PD'};
if ~ismember(controllerType, validControllers)
    error('Invalid controller type. Choose ''PID'', ''PI'', or ''PD''.');
end

% Calculation of damping ratio from maximum overshoot (eqn 36)
delta = (log(1/M_p)) / (sqrt((pi^2) + (log(1/M_p)^2))); 

% calculation of gain cut-off frequency from settling time 5% (eqn 35) [rad/s]
omega_gc = 3/(delta*t_s);
% found online 
% omega_gc = 4/(t_s);


% phase margin calculation from damping ratio (eqn 41)
phase_margin = atan( (2*delta) / (sqrt(sqrt(1 + (4*(delta^4))) - (2*(delta^2)))));

% Compute magnitude and phase of the plant at omega_gc (eqn 44)
[mag, phase] = bode(Ps, omega_gc);
mag = mag(:);
phase = phase(:);

% Required change of phase and gain, lead or lag
delta_K = (1./mag); % absolute units ? confirm this, if we want in dB then 20log(1./mag)
delta_phi = -pi + phase_margin - (deg2rad(phase)); 

% Initialize gains
Kp = 0;
Ki = 0;
Kd = 0;
Tl = 1/(2*omega_gc); % eqn 53 


% Compute gains based on controller type
switch controllerType
    case 'PID' % eqn 48
        Kp = (delta_K)*cos(delta_phi); 
        Td = (tan(delta_phi) + sqrt( (tan(delta_phi)^2) + (4/alpha) ) ) / (2*omega_gc); % eqn 49
        Ti = alpha*Td; 
        Kd = Kp*Td; 
        Ki = Kp/Ti;

    case 'PI' % eqn 52
        Kd = 0;
        Tl = 0; 
        Kp = (delta_K)*cos(delta_phi); 
        Ki = -(omega_gc)*(delta_K)*sin(delta_phi);


    case 'PD' % eqn 51
        Ki = 0; 
        Kp = (delta_K)*cos(delta_phi); 
        Kd = ((delta_K)*sin(delta_phi))/omega_gc; 
end


% Output the controller gains as a struct
pidGains = struct('Kp', Kp, 'Ki', Ki, 'Kd', Kd, 'Tl', Tl);

end

