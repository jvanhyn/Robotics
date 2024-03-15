function [Vb,du,dtheta] = FeedbackControl(q,X,Xd,Xd1,Kp,Ki,dt)

for i = 1:length(X(1,1,:))
brac_Xe(:,:,i) = MatrixLog6((X(:,:,i))\Xd);
Xe(:,i) = se3ToVec(brac_Xe(:,:,i));                  % Feedback Error-Twist
end                   

brac_Vd = 1/dt * MatrixLog6(inv(Xd)*Xd1);           % FeedForward Target-TwistFeedForward Target-Twist
Vd = se3ToVec(brac_Vd);            

control_ff = Adjoint(inv(X(:,:,end))*Xd)*Vd;
control_p =  Kp*Xe(:,end);
control_i = Ki*sum(dt*Xe,2);


Vb = control_ff + control_p + control_i;   % Commanded Control-Twist


% % JVH: Here is a proposed method for the control

% %{
%     1.  If the ee is inside some safe radius from the chasis, move the ee
%         using manipulator arm joint only in the direction of the (x,y,z) position of the goal

%     2.  If the ee is outside of the safe radius, move the wheels in the direction of the (x,y)
%         position of the goal

%     3.  Repeat steps one and two until you arrive
    
% %}

% Default configurations
M_0e =  [1 0 0 0.1667; 0 1 0 0; 0 0 1 0.6546; 0 0 0 1]; % default end effector configuration
T_b0 =  [1 0 0 0.1667; 0 1 0 0; 0 0 1 0.0330; 0 0 0 1]; % manipulator base to body frame transformation

% Manipulator screw axes
B1 = [0,  0,  1,  0,      0.0330,  0]';
B2 = [0, -1,  0, -0.5076,      0,  0]';
B3 = [0, -1,  0, -0.3526,      0,  0]';
B4 = [0, -1,  0, -0.2176,      0,  0]';
B5 = [0,  0,  1,       0,      0,  0]';
Blist = [B1,B2,B3,B4,B5];

% End-effector to manipulator base transformation
T_0e = @(q) M_0e*expm(VecTose3(B1)*q(1))*expm(VecTose3(B2)*q(2))*expm(VecTose3(B3)*q(3))*expm(VecTose3(B4)*q(4))*expm(VecTose3(B5)*q(5));

l = 0.47/2;
w = 0.3/2;
r = 0.0475;

F6 = r/4 * [0 0 0 0; 0 0 0 0; -1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w); 1 1 1 1; -1 1 -1 1; 0 0 0 0]; % F for a 4-mecanum-wheel chassis (see eqn 13.33 from mr)

theta = q(4:end);

Jbase = Adjoint(inv(T_0e(theta))*inv(T_b0))*F6;

Jm = JacobianBody(Blist,theta);

J = [Jbase,Jm];
Q = pinv(J,1e-3)*Vb;

dtheta = Q(5:end);
du = Q(1:4);

%% Solutions
% check1 = [0;0;0;20;0;10] - round(Vd,3)
% check2 = [0;0;0;21.409;0;6.455] - round(control_ff,3)
% check3 = [0;0;0;21.409;0;6.455] - round(Vb,3)
% check4 = [0;0.171;0;0.080;0;0.107] - round(Xe(:,end),3)
% check5 = [157.2,157.2,157.2,157.2,0,-652.9,1398.6,-745.7,0]' - [du;dtheta]
% check6 = [0, 0.171, 0, 21.488, 0, 6.562]' - Vb
% check7 = [157.5, 157.5, 157.5, 157.5, 0, -654.3, 1400.9, -746.8, 0]' - [du;dtheta]


end