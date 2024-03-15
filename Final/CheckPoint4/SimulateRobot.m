close all; clear; clc;

addpath(cd,'../mr')
addpath(cd,'../CheckPoint1')
addpath(cd,'../CheckPoint2')
addpath(cd,'../CheckPoint3')


% Default Configurations 
M_0e =  [1 0 0 0.1667; 0 1 0 0; 0 0 1 0.6546; 0 0 0 1]; % default end effector configuration
T_b0 =  [1 0 0 0.1667; 0 1 0 0; 0 0 1 0.0330; 0 0 0 1]; % manipulator base to body frame transformation

% Body to Spacial frame transformation
T_sb = @(phi,x,y) [cos(phi)  -sin(phi)  0    x;
                   sin(phi)  cos(phi)   0    y;
                   0         0          1    0.0963;
                   0         0          0    1];
% Manipulator screw axes
B1 = [0,  0,  1,  0,      0.0330,  0]';
B2 = [0, -1,  0, -0.5076,      0,  0]';
B3 = [0, -1,  0, -0.3526,      0,  0]';
B4 = [0, -1,  0, -0.2176,      0,  0]';
B5 = [0,  0,  1,       0,      0,  0]';

% End-effector to manipulator base transformation
T_0e = @(q) M_0e*expm(VecTose3(B1)*q(1))*expm(VecTose3(B2)*q(2))*expm(VecTose3(B3)*q(3))*expm(VecTose3(B4)*q(4))*expm(VecTose3(B5)*q(5));

% End-effector to space transformation
T_se = @(phi,x,y,q) T_sb(phi,x,y) * T_b0 * T_0e(q);

% Rotation matrix helper functions
Tz = @(x,y,z,phi) [cos(phi)  -sin(phi) 0 x;
                  sin(phi)   cos(phi) 0 y;
                  0          0        1 z;
                  0          0        0 1];
Ty = @(x,y,z,phi) [cos(phi)  0 sin(phi) x;
                   0         1      0   y;
                   -sin(phi) 0 cos(phi) z;
                   0         0      0   1];
%

u0 = [0,0,0,0]';
q0 = [0,0,0]';                    % initial config of {e}
theta0 = [0,0,0,-pi/2,0]';

Tse_i = T_se(q0(1),q0(2),q0(3),theta0);     % initial pos of {e}

Tsc_i = Tz( 1,  0,  0,     0);      % initial pos of {c}
Tsc_f = Tz( 0, -1,  0, -pi/2);      % final pos of {c}

Tce_g =  Ty( 0, 0,   0, pi);        % grasp pos of {e} wrt {c}
Tce_s = Ty(0,0,.25,pi);             % standoff pos of {e} wrt {c}

k = 1;
dt = 0.01;
speed_max = 1000000;

trajectory = TrajectoryGenerator(Tse_i,Tsc_i,Tsc_f,Tce_g,Tce_s,k);

q = q0;
theta = theta0;
u = u0;

Kp = 3*eye(6);
Ki = 0.01*eye(6);

N = length(trajectory(:,1));
Q = zeros(N,13);

for i = 1:N-1
X(:,:,i) = T_se(q(1),q(2),q(3),theta);
Xd = [trajectory(i,1:3),trajectory(i,10);trajectory(i,4:6),trajectory(i,11);trajectory(i,7:9),trajectory(i,12);0,0,0,1];
Xd_n = [trajectory(i+1,1:3),trajectory(i+1,10);trajectory(i+1,4:6),trajectory(i+1,11);trajectory(i+1,7:9),trajectory(i+1,12);0,0,0,1];

[Vb,du,dtheta] = FeedbackControl([q;theta],X,Xd,Xd_n,Kp,Ki,dt);
[q,theta,u] = NextState(q,u,theta,du,dtheta,dt,speed_max);
Q(i,:) = [q;u;theta;0]';
end

writematrix(Q,'robotmotion.csv')