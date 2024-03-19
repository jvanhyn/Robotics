close all; clear; clc;

addpath(cd,'../mr')
addpath(cd,'../CheckPoint1')
addpath(cd,'../CheckPoint2')
addpath(cd,'../CheckPoint3')

%% SimulateRobot
%{
    SimulateRobot simulates the mobile manipulation of a cube using of a KUKA youBot. 
    The youBot follows a predetermined trajectorty 
    This script uses the following functions

    -> TrajectoryGenerator: 
        takes waypoints 
        generates a desired trajectory 

    -> FeedbackControl: 
        takes desired trajectory 
        generates motor velocity commands

    -> NextState: 
        takes velocity commands 
        generates change in position over timestep dt
%}

% Helper Transformation Matrix Functions
Tx = @(x,y,z,phi) [1         0        0        x; 0         cos(phi) -sin(phi) y; 0         sin(phi) cos(phi) z; 0 0 0 1]; 
Ty = @(x,y,z,phi) [cos(phi)  0        sin(phi) x; 0         1        0         y; -sin(phi) 0        cos(phi) z; 0 0 0 1];
Tz = @(x,y,z,phi) [cos(phi) -sin(phi) 0        x; sin(phi)  cos(phi) 0         y; 0         0        1        z; 0 0 0 1];

%% Robot Info
% Reference Frames
% End-Effector      {e}
% Base              {0}
% Body              {b}
% Space             {s}

% Default Configurations 
M0e =  [1 0 0 0.0330; 0 1 0 0; 0 0 1 0.6546; 0 0 0 1]; % default End-Effector configuration
Tb0 =  [1 0 0 0.1667; 0 1 0 0; 0 0 1 0.0026; 0 0 0 1]; % Base to Body frame transformation

% Manipulator screw axes
B1 = [0,  0,  1,  0,      0.0330,  0]';
B2 = [0, -1,  0, -0.5076,      0,  0]';
B3 = [0, -1,  0, -0.3526,      0,  0]';
B4 = [0, -1,  0, -0.2176,      0,  0]';
B5 = [0,  0,  1,       0,      0,  0]';
Blist = [B1,B2,B3,B4,B5];

% Chasis z-height
zsb = 0.0963;

% Joint Velocity Limmits
speed_max = 1000*[1,1,1,1,1,1,1,1,1];

%% Initial conditions
q0 = [0,0,0]';                      % position of body (x,y,phi)         
u0 = [0,0,0,0]';                    % wheel angles     (R^4)
theta0 = [0,pi/4,-pi/4,-pi/2,0]';   % joint angles     (R^5)

%% Generate Trajectory
Tsb = Tz(q0(2),q0(3),zsb,q0(1));       % Body to Spacial frame transformation
T0e = FKinBody(M0e,Blist,theta0);      % End-Effector to Base transformation
Tse = Tsb * Tb0 * T0e;                 % End-Effector to Space transformation

% Waypoints
Tse_i = Tse;                        % initial   configuration of the end-effector   (space frame)
Tce_s = Ty( 0, 0, .25, pi/2);         % standoff  configuration of the end-effector   (cube frame)
Tce_g = Ty( 0.01, 0,  0.02, pi/2);         % grasping  configuration of the end-effector   (cube frame)
Tsc_i = Tz( 1,  0,  0,  0);      % initial   configuration of the cube           (space frame)
Tsc_f = Tz( 0, -1,  0, -pi/2);      % final     configuration of the cube           (space frame)

% Resolution
dt = 0.01;                          % time step in seconds
k = 10;                             % frames per time step

% Generate Trajectory
trajectory = TrajectoryGenerator(Tse_i,Tsc_i,Tsc_f,Tce_g,Tce_s,k,dt);

%% Control Gains
Kp = 100*eye(6);
Ki = 0.5*eye(6);

%% Running The Simulation
% Setting the initial conditions
q = q0;
u = u0;
theta = theta0;

% Loop Variables
N = length(trajectory(:,1));    % Number of simulation steps
t = linspace(0,(N-1)*dt,N-1)/k; % Simulation time
Q = zeros(N,13);                % Robot configuration
Xi = zeros(6,1);                % Integrated error twist

for i = 1:N-1
T0e = FKinBody(M0e,Blist,theta);
Tsb = [cos(q(1)) -sin(q(1)) 0 q(2); sin(q(1))  cos(q(1)) 0 q(3); 0 0 1 0.0963; 0 0 0 1]; 
Tse = Tsb * Tb0 * T0e;

Xd = [trajectory(i,1:3),trajectory(i,10);trajectory(i,4:6),trajectory(i,11);trajectory(i,7:9),trajectory(i,12);0,0,0,1];
Xd_n = [trajectory(i+1,1:3),trajectory(i+1,10);trajectory(i+1,4:6),trajectory(i+1,11);trajectory(i+1,7:9),trajectory(i+1,12);0,0,0,1];

[du,dtheta,Vb,Xe(:,i),Xi] = FeedbackControl(theta,Tse,Xd,Xd_n,Kp,Ki,Xi,dt);
[q,theta,u] = NextState(q,u,theta,du,dtheta,dt,speed_max);

Q(i,:) = [q;theta;u;trajectory(i,13)]'; 

end

writematrix(Q,'robotmotion.csv')

%% Plotting Error
figure(1)
set(gcf, 'Position', [10 10 800 400]);
tgl = tiledlayout(1,2);
title(tgl,"Error Twist vs. Time")
subtitle(tgl,"Feed Forward + Kp = 100 + Ki = 0.5")

nexttile(tgl,2)
plot(t,Xe(4:6,:)',"LineWidth",2)
legend('vx','vy','vz','Location','northwest')
title('Linear Velocity')
xlabel("Time (s)")
ylabel("Velocity (m/s)")
lx = xlim;
ly = ylim;
grid on

nexttile(tgl,1)
plot(t,Xe(1:3,:)',"LineWidth",2)
legend('wx','wy','wz','Location','northwest');
title('Rotational Velocity')
xlabel("Time (s)")
ylabel("Angular Velocity (rad/s)")
xlim(lx);
ylim(ly);
grid on