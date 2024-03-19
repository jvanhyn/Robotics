function [theta_logical_val] = testJointLimits(theta)
% testJointLimits will test whether the joints 3 and 4 in the robot arm obey the
% defined joint limits

% INPUT:
% theta: original joint angles, in the set of R5

% OUTPUT:
% theta_logical_val: an R5 vector containing logical values to determine
% whether each element of theta is within its joint limits

if abs(theta(1)) > deg2rad(169)
    theta(1) = false;
else
    theta(1) = true;
end

if theta(2) > deg2rad(90) || theta(2) < deg2rad(-65)
    theta(2) = false;
else
    theta(2) = true;
end

if theta(3) > deg2rad(146) || theta(3) < deg2rad(-150)
    theta(3) = false;
else
    theta(3) = true;
end
if abs(theta(4)) > deg2rad(102.5)
    theta(4) = false;
else
    theta(4) = true;
end
if abs(theta(5)) > deg2rad(167.5)
    theta(5) = false;
else
    theta(5) = true;
end

theta_logical_val = [theta(1); theta(2); theta(3); theta(4); theta(5)];
end