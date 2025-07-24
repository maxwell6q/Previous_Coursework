% Script to load parameters for the Selfmade_model_robot.slx model

% load the "static" parameters
balrob_params

% Voltage to torque transformation
ua2tau = (2*gbox.N*mot.Kt/mot.R)*[1;-1];

% Inertia Matrix
model.M11 = 2*wheel.Iyy + ...
            2*gbox.N^2*mot.rot.Iyy + ...
            (body.m+2*wheel.m+2*mot.rot.m)*wheel.r^2;
model.M22 = body.Iyy + ...
            2*(1-gbox.N)^2*mot.rot.Iyy + ...
            body.m*body.zb^2 +...
            2*mot.rot.m*mot.rot.zb^2;

model.M12_add = 2*gbox.N*(1-gbox.N)*mot.rot.Iyy;
model.M12_fact = (body.m*body.zb + 2*mot.rot.m*mot.rot.zb)*wheel.r;


% Centrifugal and Coriolis Matrix
model.C12_fact = -(body.m*body.zb+2*mot.rot.m*mot.rot.zb)*wheel.r;

% Viscous friction coefficients
model.Fv = [2*(gbox.B+wheel.B), -2*gbox.B;
                -2*gbox.B,       2*gbox.B];
model.Fv_ = model.Fv+ (2*gbox.N^2*mot.Kt*mot.Ke)/mot.R *[1,-1;-1,1];

% Gravity vector
model.g2_fact = -(body.m*body.zb + 2*mot.rot.m*mot.rot.zb)*g;


