function trajectory = TrajectoryGenerator(Tse_i,Tsc_i,Tsc_f,Tce_g,Tce_s,k,dt,path)
% TrajectoryGenerator creates the reference trajectory intended for the
% end-effector to follow.

% INPUTS:
% Tse_i: initial configuration of the end-effector wrt space frame, {s}
% Tsc_i: initial configuration of the object to be grasped wrt space frame, {s}
% Tsc_f: desired final configuration of the object to be grasped wrt space frame, {s}
% Tce_g: configuration of the end-effector relative to the object while grasping
% Tce_s: configuration of the end-effector above the object before and after grasping it
% k: number of trajectory reference configurations per .01 seconds

% OUPUT:
% This function will output a .csv file with the entire reference
% trajectory represented as N = t*k/0.01 configurations.  Each row in
% the .csv file will correspond to one configuration, Tse, per instant in
% time where each variable is expressed as follows: r11, r12, r13, r21,
% r22, r23, r31, r32, r33, px, py, pz, gripper_state, where the gripper
% state = 0 for open and 1 for closed.

% Example Use
% % Rotation matrix helper functions
% Tx = @(x,y,z,phi) [1         0        0        x; 0         cos(phi) -sin(phi) y; 0         sin(phi) cos(phi) z; 0 0 0 1]; 
% Ty = @(x,y,z,phi) [cos(phi)  0        sin(phi) x; 0         1        0         y; -sin(phi) 0        cos(phi) z; 0 0 0 1];
% Tz = @(x,y,z,phi) [cos(phi) -sin(phi) 0        x; sin(phi)  cos(phi) 0         y; 0         0        1        z; 0 0 0 1];

% % Folder to save .csv
% cvsfolder = pwd + "/csv/";

% % Trajectory waypoints
% Tse_i = Ty(.3334, 0, .7839, pi/2);  % initial   configuration of the end-effector   (space frame)
% Tce_s = Ty( 0, 0, .25, pi);         % standoff  configuration of the end-effector   (cube frame)
% Tce_g = Ty( 0, 0,   0, pi);         % grasping  configuration of the end-effector   (cube frame)
% Tsc_i = Tz( 1,  0,  0,     0);      % initial   configuration of the cube           (space frame)
% Tsc_f = Tz( 0, -1,  0, -pi/2);      % final     configuration of the cube           (space frame)

% % Resolution
% dt = 0.01;  % time step in seconds
% k = 10;     % frames per time step

% % Generate Trajectory
% trajectory = TrajectoryGenerator(Tse_i,Tsc_i,Tsc_f,Tce_g,Tce_s,k,dt,cvsfolder);

N = k/dt; % Steps

% Define end effector configurations for each waypoint
Tse_gi = Tsc_i*Tce_g; % grasp initial configuration
Tse_gf = Tsc_f*Tce_g; % grasp final configuration
Tse_si = Tsc_i*Tce_s; % standoff initial configuration
Tse_sf = Tsc_f*Tce_s; % standoff final configuration

% Define trajectory segments
Traj1 = CartesianTrajectory(Tse_i,Tse_si,1,N,3);    % Initial      to     Standby-1
Traj2 = CartesianTrajectory(Tse_si,Tse_gi,1,N,3);   % Standby-1    to     Cube-Initial
Traj3 = CartesianTrajectory(Tse_gi,Tse_gi,1,63,3);  % Cube-Initial to     Cube-Initial
Traj4 = CartesianTrajectory(Tse_gi,Tse_si,1,N,3);   % Cube-Initial to     Standby-1
Traj5 = CartesianTrajectory(Tse_si,Tse_sf,1,N,3);   % Standby-1    to     Standby-2
Traj6 = CartesianTrajectory(Tse_sf,Tse_gf,1,N,3);   % Standby-2    to     Cube-Final
Traj7 = CartesianTrajectory(Tse_gf,Tse_gf,1,63,3);  % Cube-Final   to     Cube-Final
Traj8 = CartesianTrajectory(Tse_gf,Tse_sf,1,N,3);   % Cube-Final   to     Standby-2

% Reformat data for CopeliaSym
for i = 1:N
    T1(i,:) = [Traj1{i}(1,1:3),Traj1{i}(2,1:3),Traj1{i}(3,1:3),Traj1{i}(1:3,4)',0];
    T2(i,:) = [Traj2{i}(1,1:3),Traj2{i}(2,1:3),Traj2{i}(3,1:3),Traj2{i}(1:3,4)',0];
    T4(i,:) = [Traj4{i}(1,1:3),Traj4{i}(2,1:3),Traj4{i}(3,1:3),Traj4{i}(1:3,4)',1];
    T5(i,:) = [Traj5{i}(1,1:3),Traj5{i}(2,1:3),Traj5{i}(3,1:3),Traj5{i}(1:3,4)',1];
    T6(i,:) = [Traj6{i}(1,1:3),Traj6{i}(2,1:3),Traj6{i}(3,1:3),Traj6{i}(1:3,4)',1];
    T8(i,:) = [Traj8{i}(1,1:3),Traj8{i}(2,1:3),Traj8{i}(3,1:3),Traj8{i}(1:3,4)',0];
end

for i = 1:63
    T3(i,:) = [Traj3{i}(1,1:3),Traj3{i}(2,1:3),Traj3{i}(3,1:3),Traj3{i}(1:3,4)',1];
    T7(i,:) = [Traj7{i}(1,1:3),Traj7{i}(2,1:3),Traj7{i}(3,1:3),Traj7{i}(1:3,4)',0];
end

% Concatanate trajectory segments
trajectory = [T1;T2;T3;T4;T5;T6;T7;T8];

% Save as .csv
writematrix(trajectory, path)

end



