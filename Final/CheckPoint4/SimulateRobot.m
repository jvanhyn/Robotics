clear; clc;

addpath(cd,'../mr')
addpath(cd,'../CheckPoint1')
addpath(cd,'../CheckPoint2')
addpath(cd,'../CheckPoint3')

%%
% Rotation matrix helper functions
Tz = @(x,y,z,phi) [cos(phi) -sin(phi) 0 x;
                  sin(phi)   cos(phi) 0 y;
                  0          0        1 z;
                  0          0        0 1];

Ty = @(x,y,z,phi) [cos(phi)  0 sin(phi) x;
                   0         1      0   y;
                   -sin(phi) 0 cos(phi) z;
                   0         0      0   1];

%% Robot Info
% Default Configurations 
M0e =  [1 0 0 0.0330; 0 1 0 0; 0 0 1 0.6546; 0 0 0 1]; % default end effector configuration
Tb0 =  [1 0 0 0.1667; 0 1 0 0; 0 0 1 0.0026; 0 0 0 1]; % manipulator base to body frame transformation

% Manipulator screw axes
B1 = [0,  0,  1,  0,      0.0330,  0]';
B2 = [0, -1,  0, -0.5076,      0,  0]';
B3 = [0, -1,  0, -0.3526,      0,  0]';
B4 = [0, -1,  0, -0.2176,      0,  0]';
B5 = [0,  0,  1,       0,      0,  0]';
Blist = [B1,B2,B3,B4,B5];

%% Initial conditions
q0 = [0,0,0]';                      % position of chasis         
u0 = [0,0,0,0]';                    % wheel angles
theta0 = [0,0,0,-pi/2,0]';           % manipulator arm angles

T0e = FKinBody(M0e,Blist,theta0); % End-effector to manipulator base transformation
Tsb = [cos(q0(3)) -sin(q0(3)) 0 q0(1); sin(q0(3))  cos(q0(3)) 0 q0(2); 0 0 1 0.0963; 0 0 0 1]; % Body to Spacial frame transformation

Tse_i = Tsb * Tb0 * T0e;            % End-effector initial position
Tsc_i = Tz( 1,  0,  0,       0);    % initial configuration of the cube
Tsc_f = Tz( 0, -1,  0,   -pi/2);    % final configuration of the cube
Tce_g = Ty( 0,  0,  0,     pi);    % grasp config of the ee wrt {c}
Tce_s = Ty( 0,  0,  .25,   -pi);    % standoff config of the ee wrt {c}

%% Trajectory Generation
k = 10;
dt = 0.01;
speed_max = 550;
trajectory = TrajectoryGenerator(Tse_i,Tsc_i,Tsc_f,Tce_g,Tce_s,k);

%% Plot the Trajectory

hold on
plot3(trajectory(:,10),trajectory(:,11),trajectory(:,12))
title("Trajectory Overview")
view(50,20)
grid on

%% Control Gains
Kp = 3*eye(6);
Ki = 0.01*eye(6);

%% Integration Variables
q = q0;
u = u0;
theta = theta0;

%% Running The Simulation
N = length(trajectory(:,1));
Q = zeros(N,13);

for i = 1:N-1
% End-effector to manipulator base transformation
T_0e = FKinBody(M0e,Blist,theta');

% Body to Spacial frame transformation
T_sb = [cos(q(3)) -sin(q(3)) 0 q(1); sin(q(3))  cos(q(3)) 0 q(2); 0 0 1 0.0963; 0 0 0 1];

% End-effector to space transformation
Tse = Tsb * Tb0 * T0e;

% X(:,:,i) = T_se(q(1),q(2),q(3),theta);
% Xd = [trajectory(i,1:3),trajectory(i,10);trajectory(i,4:6),trajectory(i,11);trajectory(i,7:9),trajectory(i,12);0,0,0,1];
% Xd_n = [trajectory(i+1,1:3),trajectory(i+1,10);trajectory(i+1,4:6),trajectory(i+1,11);trajectory(i+1,7:9),trajectory(i+1,12);0,0,0,1];
% [Vb,du,dtheta] = FeedbackControl([q;theta],X,Xd,Xd_n,Kp,Ki,dt);
% [q,theta,u] = NextState(q,u,theta,du,dtheta,dt,speed_max);
% Q(i,:) = [q;u;theta;0]';
end

writematrix(Q,'robotmotion.csv')
