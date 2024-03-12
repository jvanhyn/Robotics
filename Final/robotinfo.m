close all; clear; clc;

%% File for storing robot parameters

% Default Configurations 
M_0e =  [1 0 0 0.1667; 0 1 0 0; 0 0 1 0.6546; 0 0 0 1]; % default end effector configuration
T_b0 =  [1 0 0 0.1667; 0 1 0 0; 0 0 1 0.0330; 0 0 0 1]; % manipulator base to body frame transformation

% Body to Spacial frame transformation
T_sb = @(phi,x,y) [cos(phi)  -sin(phi)  0    x;
                   sin(phi)  cos(phi)   0    y;
                   0         0          1    0.0963;
                   0         0          0    1];
% Manipulator screw axes
B1 = [0,  0,  1,  0,      0.0330,  0]';
B2 = [0, -1,  0, -0.5076,      0,  0]';
B3 = [0, -1,  0, -0.3526,      0,  0]';
B4 = [0, -1,  0, -0.2176,      0,  0]';
B5 = [0,  0,  1,       0,      0,  0]';

% End-effector to manipulator base transformation
T_0e = @(q) M_0e*expm(VecTose3(B1)*q(1))*expm(VecTose3(B2)*q(2))*expm(VecTose3(B3)*q(3))*expm(VecTose3(B4)*q(4))*expm(VecTose3(B5)*q(5));

% End-effector to space transformation
T_se = @(phi,x,y,q) T_sb(phi,x,y) * T_b0 * T_0e(q);

save('robotinfo')



