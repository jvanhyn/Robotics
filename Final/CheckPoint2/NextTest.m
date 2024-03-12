close all; clear; clc

%% Test the NextState function

% time variables
t0 = 0;
tf = 2;
dt = 0.01;

N = cast((tf - t0)/dt,"uint8");
t = linspace(t0,tf,N);

% initial robot configuration
q0 = [0,0,0]';
u0 = [0,0,0,0]';
theta0 = [0,0,0,0,0]';

gripClosed = false;

% maximum motor angular velocity
speed_max = 10;

% constant control inputs
du = -10*[-0.5,1,1,-0.5]';
dtheta = [0,0,0,10,0]';

% preallocate vectors for use in simulation loop 
q = zeros(3,N);
u = zeros(4,N);
theta = zeros(5,N);

% Setting initial conditions for use in simulation loop 
q(:,1) = q0;
u(:,1) = u0;
theta(:,1) = theta0;

% Setting format for output to Copellia Sym
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


