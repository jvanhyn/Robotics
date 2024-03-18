clear all; close all; clc;
addpath(cd,'../mr') % Add path to Modern Robotics scripts

%% TrajectoryGenerator Test script
% Generates a .csv file to test that the end-effector moves through the
% desired trajectort on CoppeliaSim, Scene 8

% Rotation matrix helper functions
Tz = @(x,y,z,phi) [cos(phi)  -sin(phi) 0 x;
                  sin(phi)   cos(phi) 0 y;
                  0          0        1 z;
                  0          0        0 1];
Ty = @(x,y,z,phi) [cos(phi)  0 sin(phi) x;
                   0         1      0   y;
                   -sin(phi) 0 cos(phi) z;
                   0         0      0   1];

Tse_i = Ty(.3334, 0, .7839, pi/2);  % initial configuration of the end effector

Tsc_i = Tz( 1,  0,  0,     0);      % initial configuration of the cube
Tsc_f = Tz( 0, -1,  0, -pi/2);      % final configuration of the cube

Tce_g = Ty( 0, 0,   0, pi);        % grasp config of the ee wrt {c}
Tce_s = Ty( 0, 0, .25, pi);        % standoff config of the ee wrt {c}

k = 10;

trajectory = TrajectoryGenerator(Tse_i,Tsc_i,Tsc_f,Tce_g,Tce_s,k);

writematrix(trajectory,"trajectory.csv")

%% Plot the Trajectory
fig1 = figure(1);
plot3(trajectory(:,10),trajectory(:,11),trajectory(:,12))
title("Trajectory Overview")
view(50,20)
grid on

%% Animate the Trajectory
run TrajectoryTestAnimation.m



