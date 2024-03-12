close all; clear; clc
addpath(cd,'../mr') % Add path to Modern Robotics scripts

%% NextState Test script
% Generates a .csv file to test that the robot chasis and end-effector 
% are simulated appopriately and behave well in CoppeliaSim, Scene 6

% Time variables
t0 = 0;     % initial time
tf = 2;     % final time
dt = 0.01;  % timestep

N = cast((tf - t0)/dt,"uint8"); % number of steps
t = linspace(t0,tf,N);          % simulation time vector

% Initial robot configuration
q0 = [0,0,0]';                  % chasis orientation and location
u0 = [0,0,0,0]';                % wheel angles
theta0 = [0,0,0,0,0]';          % manipulator arm joint angles

gripClosed = false;             % gripper state

% Maximum motor angular velocity
speed_max = 10;

% Constant control inputs
du = -10*[-0.5,1,1,-0.5]';      % wheel velocities
dtheta = [0,0,0,10,0]';         % manipulator arm joint velocities

% Preallocate state vectors for use in simulation loop 
q = zeros(3,N);
u = zeros(4,N);
theta = zeros(5,N);

% Setting initial the conditions 
q(:,1) = q0;
u(:,1) = u0;
theta(:,1) = theta0;

% Setting format for output to CopelliaSim
Q(1,:) = [q(:,1);theta(:,1);u(:,1);gripClosed]';

%% Running the simulation
for i = 1:N
    [q(:,i+1),theta(:,i+1),u(:,i+1)] = NextState(q(:,i),u(:,i),theta(:,i),du,dtheta,dt,speed_max);
    Q(i+1,:) = [q(:,i+1);theta(:,i+1);u(:,i+1);gripClosed]';
end

%% Export data
writematrix(Q,'robotMotion.csv')

%% Plot results
run NextTestAnimation.m


