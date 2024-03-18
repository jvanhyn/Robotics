% Cleear the Workspace
clear all; close all; clc;

% Add path to Modern Robotics scripts
addpath(cd,'../mr') 

%% TrajectoryGenerator Test script
% Generates a .csv file to test that the end-effector moves through the
% desired trajectort on CoppeliaSim, Scene 8

% Rotation matrix helper functions
Tx = @(x,y,z,phi) [1         0        0        x; 0         cos(phi) -sin(phi) y; 0         sin(phi) cos(phi) z; 0 0 0 1]; 
Ty = @(x,y,z,phi) [cos(phi)  0        sin(phi) x; 0         1        0         y; -sin(phi) 0        cos(phi) z; 0 0 0 1];
Tz = @(x,y,z,phi) [cos(phi) -sin(phi) 0        x; sin(phi)  cos(phi) 0         y; 0         0        1        z; 0 0 0 1];

% Trajectory waypoints
Tse_i = Ty(.3334, 0, .7839, pi/2);  % initial   configuration of the end-effector   (space frame)
Tce_s = Ty( 0, 0, .25, pi);         % standoff  configuration of the end-effector   (cube frame)
Tce_g = Ty( 0, 0,   0, pi);         % grasping  configuration of the end-effector   (cube frame)
Tsc_i = Tz( 1,  0,  0,     0);      % initial   configuration of the cube           (space frame)
Tsc_f = Tz( 0, -1,  0, -pi/2);      % final     configuration of the cube           (space frame)

% Resolution
dt = 0.01;  % time step in seconds
k = 10;     % frames per time step

% Generate Trajectory
trajectory = TrajectoryGenerator(Tse_i,Tsc_i,Tsc_f,Tce_g,Tce_s,k,dt);

% Save as .csv
writematrix(trajectory,"trajectory.csv")

%% Animate the Trajectory
run TrajectoryTestAnimation.m



