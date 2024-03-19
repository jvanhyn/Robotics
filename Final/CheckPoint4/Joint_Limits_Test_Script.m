clear all; close all; clc;
% Test Joint Limits to use in the SimulateRobot function

% Initial values
q = [0,0,0]';                      % position of chasis         
u = [0,0,0,0]';                    % wheel angles
theta = [.1,pi/4,-pi/4,-pi/2,.1]';              % manipulator arm angles
du = [0,0,0,0]'; 
dtheta = [0,0,0,0,0]'; 

dt = 0.01;
speed_max = 200;
i = 1;
max_iterations = 1000;  % Set the maximum number of iterations

% Add Joint Limits to prevent collisions
while ~any(testJointLimits(theta)) == false || i <= max_iterations % Check both conditions
    [q,theta1,u] = NextState(q,u,theta,du,dtheta,dt,speed_max); % parameters of the next state
    theta = theta1;
    i = i+1;
    theta_logical(:,i) = testJointLimits(theta); % store logical values
end