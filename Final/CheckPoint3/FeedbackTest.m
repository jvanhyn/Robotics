close all; clear; clc; 
%addpath(cd,'../mr')
%% FeedbackControl test script

% Current Desired Robot Position
Xd = [...
    0   0   1   0.5   ;
    0   1   0   0     ;
    -1  0   0   0.5   ;
    0   0   0   1     ;
    ];
%

% Next Desired Robot Position
Xd1 = [...
    0   0   1   0.6   ;
    0   1   0   0     ;
    -1  0   0   0.3   ;
    0   0   0   1     ;
    ];
%

% Current Robot Position 
X = [...
    0.170   0   0.985   0.387   ;
    0       1   0       0       ;
    -0.985  0   0.170   0.570   ;
    0       0   0       1       ;
    ];

dt = 0.01; % time step

Kp = zeros(6);
Ki = zeros(6);

q = [0, 0, 0, 0, 0, 0.2, -1.6, 0]; % robot arm config
theta = q(4:end)';
%[Vb,du,dtheta] = FeedbackControl(theta,X,Xd_i,Xd_f,Kp,Ki,dt);

%% Testing the feedback control function with joint limits
for i = 1:length(X(1,1,:))
brac_Xe(:,:,i) = MatrixLog6((X(:,:,i))\Xd);
%Xe(:,i) = round(se3ToVec(brac_Xe(:,:,i)),3);                  % Feedback Error-Twist
Xe(:,i) = se3ToVec(brac_Xe(:,:,i));
end  
Xe = Xe(:,end);

brac_Vd = 1/dt * MatrixLog6(Xd\Xd1);           % FeedForward Target-Twist
Vd = se3ToVec(brac_Vd);            

control_ff = Adjoint(X(:,:,end)\Xd)*Vd;
control_p =  Kp*Xe(:,end);
control_i = Ki*sum(dt*Xe,2);

Vb = control_ff + control_p + control_i;   % Commanded Control-Twist

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

% calculate theta
eomg = .1; %error tolerances
ev = .1;

theta_logical_val = testJointLimits(theta)

% insert testJointLimits function here to make sure theta doesn't violate it
% i = 0;
% maxiterations = 50;
% while ~any(theta_logical_val) == false && i < maxiterations
%     [theta, success] = IKinBodyIterations_1(Blist, M_0e, Xd1, theta, eomg, ev) %should it be Xd or Xd_n?
%     fprintf(theta)
%     i = i+1;
%     theta_logical_val = testJointLimits(theta);
% end
theta_vals = [];
while ~any(theta_logical_val) == false
    %recalculate theta
    [theta, success] = IKinBody(Blist, M_0e, Xd1, theta, eomg, ev);
    theta_logical_value = testJointLimits(theta);
    theta_vals = theta;
end

%theta = theta(:,end);
logic = testJointLimits(theta_vals)

% Jbase = Adjoint(inv(T_0e(theta))*inv(T_b0))*F6;

% Jm = JacobianBody(Blist,theta);
Jbase = Adjoint(inv(T_0e(theta_vals))*inv(T_b0))*F6;

Jm = JacobianBody(Blist,theta_vals);

J = [Jbase,Jm];
%J = round(J,3);
dq = pinv(J,1e-3)*Vb; % new config

dtheta = dq(5:end);
du = dq(1:4);

%% Solutions check
check1_Vd = [0;0;0;20;0;10] - round(Vd,3)
check2_Ad_Vd = [0;0;0;21.409;0;6.455] - round(control_ff,3)
check3_Vb = [0;0;0;21.409;0;6.455] - round(Vb,3)
check4_Xe = [0;0.171;0;0.080;0;0.107] - round(Xe(:,end),3)
check5_du_dtheta = [157.2,157.2,157.2,157.2,0,-652.9,1398.6,-745.7,0]' - [du;dtheta]
% check6_Vb_Kp_I = [0, 0.171, 0, 21.488, 0, 6.562]' - Vb
% check7_du_dtheta_Kp_I = [157.5, 157.5, 157.5, 157.5, 0, -654.3, 1400.9, -746.8, 0]' - [du;dtheta]
check8_J = ([.030 -.030 -.030  .030 -.985     0     0     0    0;
             0     0     0     0     0     -1    -1    -1    0;
          -.005  .005  .005 -.005  .170     0     0     0    1;
           .002  .002  .002  .002    0   -.240 -.214 -.218   0;
          -.024  .024    0     0   .221     0     0     0    0;
           .012  .012  .012  .012    0   -.288 -.135    0    0] - round(J,3))



