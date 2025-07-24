% Choose to load parameters estimated from the motor or nominal

% Motor Friction (lab0)
mld.Beq = 2.5663e-6;    % [Nm/(rad/s)]
mld.tausf = 0.013;      % [Nm]

if sIn.motor_or_nominal == 0   
    % Resonant Load (lab3)
    mld.Bb = 0.00579 ;
    mld.k = 0.73030;
end