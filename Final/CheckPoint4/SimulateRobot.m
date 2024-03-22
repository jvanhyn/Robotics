
addpath(cd,'../mr')
addpath(cd,'../CheckPoint1')
addpath(cd,'../CheckPoint2')
addpath(cd,'../CheckPoint3')

%% SimulateRobot
%{
    SimulateRobot simulates the mobile manipulation of a cube using a KUKA youBot.
    A Feed Forward + PI contoller guids the robot along a pregenerated trajectory.
    The trajectory outlines a path from the robot to initial and final positions of the cube.
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


%% Running The Simulation
% Setting the initial conditions
q = q0;
u = u0;
theta = theta0;

% Loop Variables
N = length(trajectory(:,1));    % Number of simulation steps
t = linspace(0,(N-1)*dt,N-1)/k; % Simulation time
Q = zeros(N-1,13);              % Robot configuration
Xe = zeros(6,N-1);              % Error in end-effector frame
Xs = zeros(6,N-1);              % Error in space frame
Xi = zeros(6,1);                % Integrated error 

for i = 1:N-1
    % Calculate curent spacial configuration
    T0e = FKinBody(M0e,Blist,theta);          % end effector to base transformation 
    Tsb = [cos(q(1)) -sin(q(1)) 0 q(2);       % body to space frame transformation
           sin(q(1))  cos(q(1)) 0 q(3);
           0 0 1 0.0963;
           0 0 0 1];

    Tse = Tsb * Tb0 * T0e;                    % end effector to space frame transformation 
    
    % desired configuration
    Xd = [trajectory(i,1:3),trajectory(i,10);trajectory(i,4:6),trajectory(i,11);trajectory(i,7:9),trajectory(i,12);0,0,0,1];

    % desired next configuration
    Xd_n = [trajectory(i+1,1:3),trajectory(i+1,10);trajectory(i+1,4:6),trajectory(i+1,11);trajectory(i+1,7:9),trajectory(i+1,12);0,0,0,1];
    
    % Feedback Coontrol
    [du,dtheta,Vb,Xe(:,i),Xi] = FeedbackControl(theta,Tse,Xd,Xd_n,Kp,Ki,Xi,dt,control);

    % Simulate over 1 timestep
    [q,theta,u] = NextState(q,u,theta,du,dtheta,dt,speed_max);
    
    % Store Solution
    Q(i,:) = [q;theta;u;trajectory(i,13)]';
    
    Xs(:,i) = Adjoint(Tse) * Xe(:,i);
end



