% function lab2(theta_1, theta_2, theta_3, theta_4, theta_5, theta_6)
% Lab 2 (Forward Kinematics)
% Computes the transformation matrix T of the end-effector using the joint
% angles of the UR3e robot
%
% Input: 6 joint angles theta_1...theta_6
%
% Output: none (but prints forward kinematics results to command window


%% Your code begins here

R0 = 88;
R1 = 152;
R2 = 120;
R3 = 244;
R4 = 93;
R5 = 213;
R6 = 104;
R7 = 85;
R8 = 92;

% First, define the rest of the screw axes, in (mm)

S1 = [0, 0, 1, -300, 0, 0]';
S2 = [0, 1, 0, -208, 0, 0]';
% S3 = 
% S4 = 
% S5 =
% S6 =


% Next, define the M matrix (the zero-position e-e transformation matrix),
% in (mm)

% M =



% Now, calculate the T matrix using forward kinematics

% T = 




%% Your code ends here

% Output results to command window
% disp('The transformation matrix T is:')
% disp(T)
% fprintf('The position of the end-effector is %4.2f, %4.2f, %4.2f (mm) \n',T(1,4),T(2,4),T(3,4))
% end