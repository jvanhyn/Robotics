function [Vb,du,dtheta] = FeedbackControl(q,theta,Tse,Tsd,Tsdn,Kp,Ki,dt)
R = norm(Tsd(1:3,4));
Ted = Tse\Tsd;
Tddn = Tsd\Tsdn;

brac_Verr = MatrixLog6(Ted);
Verr = se3ToVec(brac_Verr);                        % Feedback Error-Twist

brac_Vd = 1/dt * MatrixLog6(Tddn);           % FeedForward Target-TwistFeedForward Target-Twist
Vd = se3ToVec(brac_Vd);            

control_ff = Adjoint(Ted)*Vd;
control_p = zeros(6,1);
control_i = zeros(6,1);

Vb = control_ff + control_p + control_i;   % Commanded Control-Twist

% Default Configurations 
M0e =  [1 0 0 0.0330; 0 1 0 0; 0 0 1 0.6546; 0 0 0 1]; % default end effector configuration
Tb0 =  [1 0 0 0.1667; 0 1 0 0; 0 0 1 0.0026; 0 0 0 1]; % manipulator base to body frame transformation

% Manipulator screw axes
B1 = [0,  0,  1,  0,      0.0330,  0]';
B2 = [0, -1,  0, -0.5076,      0,  0]';
B3 = [0, -1,  0, -0.3526,      0,  0]';
B4 = [0, -1,  0, -0.2176,      0,  0]';
B5 = [0,  0,  1,       0,      0,  0]';
Blist = [B1,B2,B3,B4,B5];

T0e = FKinBody(M0e,Blist,theta); % End-effector to manipulator base transformation

l = 0.47/2;
w = 0.3/2;
r = 0.0475;

F6 = r/4 * [0 0 0 0; 0 0 0 0; -1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w); 1 1 1 1; -1 1 -1 1; 0 0 0 0]; % F for a 4-mecanum-wheel chassis (see eqn 13.33 from mr)

% Joint Limits as Singularity Control
if abs(theta(1)) > deg2rad(169)
    theta(1) = deg2rad(169);
end
if theta(2) > deg2rad(90) || theta(2) < deg2rad(-65)
    theta(2) = deg2rad(90);
end

if theta(3) > deg2rad(146) || theta(3) < deg2rad(-150)
    theta(3) = deg2rad(146);
end
if abs(theta(4)) > deg2rad(102.5)
    theta(4) = deg2rad(102.5);
end
if abs(theta(5)) > deg2rad(167.5)
    theta(5) = deg2rad(167.5);
end

Jbase = Adjoint(inv(T0e)*inv(Tb0))*F6;
Jm = JacobianBody(Blist,theta);

J = [Jbase,Jm];
Q = pinv(J,1e-3)*Vb;
dtheta = Q(5:end);
du = Q(1:4);



% Singularity check
% if rank(Jm,1e-3) < 5
%     dtheta = zeros(5,1);
% end


%% Solutions
% check1 = [0;0;0;20;0;10] - round(Vd,3)
% check2 = [0;0;0;21.409;0;6.455] - round(control_ff,3)
% check3 = [0;0;0;21.409;0;6.455] - round(Vb,3)
% check4 = [0;0.171;0;0.080;0;0.107] - round(Xe(:,end),3)
% check5 = [157.2,157.2,157.2,157.2,0,-652.9,1398.6,-745.7,0]' - [du;dtheta]
% check6 = [0, 0.171, 0, 21.488, 0, 6.562]' - Vb
% check7 = [157.5, 157.5, 157.5, 157.5, 0, -654.3, 1400.9, -746.8, 0]' - [du;dtheta]
end