function [Vb,du,dtheta] = FeedbackControl(X,Xd,Xd1,Kp,Ki,dt)
% brac_Xe = matrixLog6(Xd\X);
% brac_Vd = 1/dt*MatrixLog6(Xd\Xd1)
% Xe = se3ToVec(brac_Xe)
% Vd = se3ToVec(brac_Vd)

% Vb = Adjoint(Xd\X)*Vd+Kp*Xe+Ki*Xe*dt
end