close all
clear
clc

% Link Lengths
l1 = 152;
l2 = 120;
l3 = 244;
l4 = 93;
l5 = 213;
l6 = 104;
l7 = 85;
l8 = 92;

% Relative Joint Positions
q1 = [0, -300,  88]'                    ;
q2 = [0,    0,  l1]'                    ;
q3 = [0,   l2,   0]' + [l3,   0,   0]'  ;
q4 = [0,  -l4,   0]' + [l5,   0,   0]'  ;
q5 = [0,   l6,   0]' + [0,    0, -l7]'  ;
q6 = [0,   l8,   0]'                    ;

% Spacial Joint Positions
sq1 =       q1;
sq2 = sq1 + q2;
sq3 = sq2 + q3;
sq4 = sq3 + q4;
sq5 = sq4 + q5;
sq6 = sq5 + q6;

% Rotation Axes
w1 = [0,0,1]';
w2 = [0,1,0]';
w3 = [0,1,0]';
w4 = [0,1,0]';
w5 = [0,0,-1]';
w6 = [0,1,0]';

% Linear Velocities
v1 = cross(-w1,sq1);
v2 = cross(-w2,sq2);
v3 = cross(-w3,sq3);
v4 = cross(-w4,sq4);
v5 = cross(-w5,sq5);
v6 = cross(-w6,sq6);

% Screw Axes
S1 = [w1;v1];
S2 = [w2;v2];
S3 = [w3;v3];
S4 = [w4;v4];
S5 = [w5;v5];
S6 = [w6;v6];
S = {S1,S2,S3,S4,S5,S6};

% Zero Position
M1 = [eye(3),sq1;0,0,0,1];
M2 = [eye(3),sq2;0,0,0,1];
M3 = [eye(3),sq3;0,0,0,1];
M4 = [eye(3),sq4;0,0,0,1];
M5 = [eye(3),sq5;0,0,0,1];
M6 = [eye(3),sq6;0,0,0,1];
M = {M1,M2,M3,M4,M5,M6};

% Joint Angles
THETA1 = [-20, -40, 60, 10, 30, 0]';
THETA2 = [5, 10, -30, 230, -50, 150]';
THETA3 = [-15, -10, -45, 30, 20, 0]';
THETA = {THETA1,THETA2,THETA3};

% Foward Kinemaatics
P_FK = jFK(S,M,THETA1);

% EE Position
THETA0 = [0,0,0,0,0,0,0]';
XYZd = [100; 100; 100];
thresh = 0.1;

% Foward Kinematics
THETA_IK = jIK(S,M,THETA0,Tsd,thresh)
