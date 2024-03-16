close all; clear; clc; 
addpath(cd,'../mr')
%% FeedbackControl test script

% Current Desired Robot Position
Xd_i = [...
    0   0   1   0.5   ;
    0   1   0   0     ;
    -1  0   0   0.5   ;
    0   0   0   1     ;
    ];
%

% Next Desired Robot Position
Xd_f = [...
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

Kp = eye(6);
Ki = zeros(6);

q = [0, 0, 0, 0, 0, 0.2, -1.6, 0]; % robot config
%[Vb,du,dtheta] = FeedbackControl(q,X,Xd_i,Xd_f,Kp,Ki,dt);

%% FeedBack Control script assessment
for i = 1:length(X(1,1,:))
brac_Xe(:,:,i) = MatrixLog6((X(:,:,i))\Xd_i);
Xe(:,i) = se3ToVec(brac_Xe(:,:,i));             % Feedback Error-Twist (calculated for each X as it updates
end 

brac_Vd = 1/dt * MatrixLog6(pinv(Xd_i)*Xd_f);   % FeedForward Target-Twist
Vd = se3ToVec(brac_Vd);            

control_ff = Adjoint(pinv(X(:,:,end))*Xd_i)*Vd;
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
theta = q(4:end); % joint angles
%T_0e = FKinBody(M_0e, Blist, theta);

l = 0.47/2;
w = 0.3/2;
r = 0.0475;

F6 = r/4 * [0 0 0 0; 0 0 0 0; -1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w); 1 1 1 1; -1 1 -1 1; 0 0 0 0]; % F (body frame twist of the chassis) for a 4-mecanum-wheel chassis (see eqn 13.33 from mr)

Jbase = Adjoint(inv(T_0e(theta))*inv(T_b0))*F6; % Jacobian of the chassis wrt the ee
%Jbase = Adjoint(inv(T_0e)*inv(T_b0))*F6;

Jarm = JacobianBody(Blist,theta);

Je = [Jbase,Jarm];
Q = pinv(Je,1e-3)*Vb;

dtheta = Q(5:end);
du = Q(1:4);

