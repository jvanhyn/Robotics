function [V,u,dtheta] = FeedbackControl(X,Xd,Xd_next,Kp,Ki,dt)

% INPUTS:
% X: actual ee config
% Xd: current reference ee config
% Xd_next: reference ee config at the next timestep
% Kp, Ki: PI gain matrices
% dt: timestep

% OUTPUts:
% V: commanded ee twist
% u: commanded wheel speeds
% dtheta: commanded joint speeds

%brac_Xerr = matrixLog6(Xd\X); % matrix representation of error twist
brac_Xerr = matrixLog6(pinv(X)*Xd); % matrix representation of error twist
brac_Vd = 1/dt*MatrixLog6(Xd); % matrix representation of desired twist
Xerr = se3ToVec(brac_Xerr); % ee position err
Vd = se3ToVec(brac_Vd); % desired twist 

Vb = Adjoint(Xd\X)*Vd+Kp*Xerr+Ki*Xerr*dt
end