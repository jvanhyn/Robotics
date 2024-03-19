function [du,dtheta,Vb,Xe,Xi] = FeedbackControl(theta,Tse,Tsd,Tsdn,Kp,Ki,Xi,dt,control)
% FeedbackControl uses target trajectory to generate commaned joint velocites.

% INPUTS:
% theta: current joint angles (in the set of R^5)

% Tse: current spacial configuration of the robot        (in the set of SE(3))
% Td:  desired spacial configuration of the robot        (in the set of SE(3))
% Tdn: desired future spacial configuration of the robot (in the set of SE(3))

% Kp: proportial control gain  (in the set of R^6x6)
% Ki: proportial integral gain (in the set of R^6x6)

% OUTPUTS: configuration of the next state
% Vb: commanded body twist                      (in the set of R^6)
% du: commanded wheel velocities                (in the set of R^4)
% theta: commanded manioulator joint velocities (in the set of R^5)

Ted = Tse\Tsd;                      % Transformation from {d} to {e}
Tddn = Tsd\Tsdn;                    % Transformation from {d} to {dn}

brac_Xe = MatrixLog6(Ted);        % Error Twist Xe as Matrix 
Xe = se3ToVec(brac_Xe);         % Error Twist Xe      

brac_Vd = 1/dt * MatrixLog6(Tddn); % Desired Twist Vd as Matrix 
Vd = se3ToVec(brac_Vd);            % Desired Twist Vd 

Xi = Xi + Xe*dt;                   % integrate the error over the next time step

control_ff = control(1)*Adjoint(Ted)*Vd;      % Feed Forward control component 
control_p = control(2)*Kp*Xe;               % Proportional control component 
control_i = control(3)*Ki*Xi;            % Integral control component 

Vb = control_ff + control_p + control_i;  % Commanded control twist

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
Jm = JacobianBody(Blist,theta);

l = 0.47/2;
w = 0.3/2;
r = 0.0475;

F6 = r/4 * [0 0 0 0; 0 0 0 0; -1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w); 1 1 1 1; -1 1 -1 1; 0 0 0 0]; % F for a 4-mecanum-wheel chassis (see eqn 13.33 from mr)
Jbase = Adjoint(inv(T0e)*inv(Tb0))*F6;

J = [Jbase,Jm];
Q = pinv(J,1e-3)*Vb;
dtheta = Q(5:end);
du = Q(1:4);

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