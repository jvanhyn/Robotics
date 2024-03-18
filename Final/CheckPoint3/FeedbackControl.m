% INPUTS:
% thetalist0: initial guess for joint angles
% X: current actual ee config
% Xd: current reference ee config
% Xd_next: reference ee config at the next timestep
% Kp, Ki: PI gain matrices
% dt: timestep

% OUTPUts:
% V: commanded ee twist
% du: commanded wheel speeds
% dtheta: commanded joint speeds
% Xe: configuration error between X and Xd

function [Vb,du,dtheta,Xe] = FeedbackControl(q,X,Xd,Xd1,Kp,Ki,dt)

for i = 1:length(X(1,1,:))
brac_Xe(:,:,i) = MatrixLog6((X(:,:,i))\Xd);                % Feedback Error-Twist
Xe(:,i) = se3ToVec(brac_Xe(:,:,i));
end  
Xe = Xe(:,end);

brac_Vd = 1/dt * MatrixLog6(Xd\Xd1);           % FeedForward Target-Twist
Vd = se3ToVec(brac_Vd);            

control_ff = Adjoint(X(:,:,end)\Xd)*Vd;
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
M_0e =  [1 0 0 0.033; 0 1 0 0; 0 0 1 0.6546; 0 0 0 1]; % default end effector configuration
T_b0 =  [1 0 0 0.1662; 0 1 0 0; 0 0 1 0.0026; 0 0 0 1]; % manipulator base to body frame transformation

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



%theta = q(4:end);

% % calculate theta
% eomg = .1; %error tolerances
% ev = .1;
% %[theta, success] = IKinBody(Blist, M_0e, Xd_n, thetalist0, eomg, ev) %should it be Xd or Xd_n?
% theta = thetalist0;
% theta_logical_val = testJointLimits(theta)

% % insert testJointLimits function here to make sure theta doesn't violate it
% i = 0;
% maxiterations = 50;
% while ~any(theta_logical_val) == false && i < maxiterations
%     [theta, success] = IKinBody(Blist, M_0e, Xd1, theta, eomg, ev) %should it be Xd or Xd_n?
%     i = i+1;
%     theta_logical_val = testJointLimits(theta);
% end

Jbase = Adjoint(inv(T_0e(q))*inv(T_b0))*F6;

Jm = JacobianBody(Blist,q);

J = [Jbase,Jm];
dq = pinv(J)*Vb; % new config

% check that joints 3 and 4 satisfy the joint limits

dtheta = dq(5:end);
du = dq(1:4);

% implement velocity limits (limits obtained from
% http://www.youbot-store.com/wiki/index.php/YouBot_Detailed_Specifications)
max_joint_speeds = [549.7, 549.7, 549.7, 298.4, 293.2]; % Maximum speeds for each joint [rad/s]
dtheta = min(dtheta, max_joint_speeds); % Ensure dtheta doesn't exceed maximum speeds
dtheta = dtheta(:,end);

max_wheel_speeds = [549.7, 549.7, 549.7, 549.7];% Maximum speeds for each chassis wheel [rad/s]
du = min(du, max_wheel_speeds);
du = du(:,end);

%% Solutions
% check1_Vd = [0;0;0;20;0;10] - round(Vd,3)
% check2_Ad_Vd = [0;0;0;21.409;0;6.455] - round(control_ff,3)
% check3_Vb = [0;0;0;21.409;0;6.455] - round(Vb,3)
% check4_Xe = [0;0.171;0;0.080;0;0.107] - round(Xe(:,end),3)
% check5_du_dtheta = [157.2,157.2,157.2,157.2,0,-652.9,1398.6,-745.7,0]' - [du;dtheta]
% % check6_Vb_Kp_I = [0, 0.171, 0, 21.488, 0, 6.562]' - Vb
% % check7_du_dtheta_Kp_I = [157.5, 157.5, 157.5, 157.5, 0, -654.3, 1400.9, -746.8, 0]' - [du;dtheta]
% check8_J = ([.030 -.030 -.030  .030 -.985     0     0     0    0;
%              0     0     0     0     0     -1    -1    -1    0;
%           -.005  .005  .005 -.005  .170     0     0     0    1;
%            .002  .002  .002  .002    0   -.240 -.214 -.218   0;
%           -.024  .024    0     0   .221     0     0     0    0;
%            .012  .012  .012  .012    0   -.288 -.135    0    0] - round(J,3))

end