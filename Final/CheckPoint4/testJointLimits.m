function [theta_logical_val] = testJointLimits(theta)
% testJointLimits will test whether the joints 3 and 4 in the robot arm obey the
% defined joint limits

% INPUT:
% theta: original joint angles, in the set of R5

% OUTPUT:
% theta_logical_val: an R5 vector containing logical values to determine
% whether each element of theta is within its joint limits

theta(1) = true;
theta(2) = true;

if theta(3) < -.2
    theta(3) = true;
else
    theta(3) = false;
end

if theta(4) < -.2
    theta(4) = true;
else
    theta(4) = false;
end

theta(5) = true;

theta_logical_val = [theta(1); theta(2); theta(3); theta(4); theta(5)];
end