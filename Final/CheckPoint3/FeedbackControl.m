function [du,dtheta,Vb,Xe,Xi] = FeedbackControl(theta,Tse,Tsd,Tsdn,Kp,Ki,Xi,dt,control,joint_lims)
% FeedbackControl uses target trajectory to generate commaned joint velocites.

% INPUTS:
% theta: current joint angles (in the set of R^5)

% Tse: current spacial configuration of the robot        (in the set of SE(3))
% Td:  desired spacial configuration of the robot        (in the set of SE(3))
% Tdn: desired future spacial configuration of the robot (in the set of SE(3))

% Kp: proportial control gain  (in the set of R^6x6)
% Ki: proportial integral gain (in the set of R^6x6)

% control: 1X3 vector of logical values that toggle 

% OUTPUTS: configuration of the next state
% Vb: commanded body twist                      (in the set of R^6)
% du: commanded wheel velocities                (in the set of R^4)
% theta: commanded manioulator joint velocities (in the set of R^5)

%% Robot Info
% Default Configurations 
M0e =  [1 0 0 0.0330; 0 1 0 0; 0 0 1 0.6546; 0 0 0 1]; % default end effector to base configuration
Tb0 =  [1 0 0 0.1667; 0 1 0 0; 0 0 1 0.0026; 0 0 0 1]; % manipulator base to body frame transformation

% Manipulator screw axes
B1 = [0,  0,  1,  0,      0.0330,  0]';
B2 = [0, -1,  0, -0.5076,      0,  0]';
B3 = [0, -1,  0, -0.3526,      0,  0]';
B4 = [0, -1,  0, -0.2176,      0,  0]';
B5 = [0,  0,  1,       0,      0,  0]';
Blist = [B1,B2,B3,B4,B5];

% manipulator effector to base transformation 
T0e = FKinBody(M0e,Blist,theta);

% Chasis dimensions 
l = 0.47/2;
w = 0.3/2;
r = 0.0475;

% F for a 4-mecanum-wheel chassis (see eqn 13.33 from mr)
F6 = r/4 * [0 0 0 0; 0 0 0 0; -1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w); 1 1 1 1; -1 1 -1 1; 0 0 0 0]; 

%% Caculate Error 
Ted = Tse\Tsd;                      % Transformation from {d} to {e}
Tddn = Tsd\Tsdn;                    % Transformation from {d} to {dn}

brac_Vd = 1/dt * MatrixLog6(Tddn);  % Desired Twist Vd as Matrix 
Vd = se3ToVec(brac_Vd);             % Desired Twist Vd 

brac_Xe = MatrixLog6(Ted);          % Error Twist Xe as Matrix 
Xe = se3ToVec(brac_Xe);             % Error Twist Xe      

Xi = Xi + Xe*dt;                    % Integral of Error Twist Xi

%% Control Law
control_ff = Adjoint(Ted)*Vd;    % Feed Forward control component 
control_p  = Kp*Xe;              % Proportional control component 
control_i  = Ki*Xi;              % Integral control component 

% Toggle control options
control_ff = control(1)*control_ff;   % toggle Feed Forward control
control_p  = control(2)*control_p;    % toggle Proportional control  
control_i  = control(3)*control_i;    % toggle Integral control 

% Commanded Twist
Vb = control_ff + control_p + control(3)*control_i;   

%% Jacobians
Jm = JacobianBody(Blist,theta);
Jbase = Adjoint(inv(T0e)*inv(Tb0))*F6;

% Enforces Joint Limit
for i = 1:4
    if(abs(theta(i)) > joint_lims(i))
        Jm(:,i) = zeros(6,1);
    end
end

% Agregate Jacobian 
J = [Jbase,Jm];

%% Joint Velocities
Q = pinv(J,1e-3)*Vb;

dtheta = Q(5:end);
du = Q(1:4);

% Backs Joint Away from Limit
for i = 1:4
    if(abs(theta(i)) > joint_lims(i))
        dtheta(i) = (-0.1)*sign(theta(i));
    end
end

end