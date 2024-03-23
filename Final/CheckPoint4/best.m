run robotinfo 

% This script contains initial conditions for and generates the desired
% trajectory for the best case control scenario

%% Robot Initial conditions
q0 = [pi/6,-0.2,0]';                   % position of body (phi,x,y)
u0 = [0,0,0,0]';                       % wheel angles     (R^4)
theta0 = [0,pi/4,-pi/4,-pi/6,0]';      % joint angles     (R^5)

%% Generate Trajectory
% Trajectory initial conditions
qi = [0,0,0]';                  % position of body (phi,x,y)
ui = [0,0,0,0]';                       % wheel angles     (R^4)
thetai = [0,pi/4,-pi/4,-pi/2,0]';      % joint angles     (R^5)

% Trajectory initial conditions
Tsb_i = Tz(qi(2),qi(3),zsb,qi(1));       % Body to Spacial frame transformation
T0e_i = FKinBody(M0e,Blist,thetai);      % End-Effector to Base transformation

% Waypoints
Tse_i = Tsb_i * Tb0 * T0e_i;           % initial   configuration of the end-effector   wrt (space frame)
Tce_s = Ty( 0, 0, .25, 2*pi/3);        % standoff  configuration of the end-effector   wrt (cube frame)
Tce_g = Ty( 0.01, 0,  0.02, 2*pi/3);   % grasping  configuration of the end-effector   wrt (cube frame)
Tsc_i = Tz( 1,  0,  0,  0);            % initial   configuration of the cube           wrt (space frame)
Tsc_f = Tz( 0, -1,  0, -pi/2);         % final     configuration of the cube           wrt (space frame)

% Resolution
dt = 0.01;                             % time step in seconds
k = 10;                                % frames per time step

% Generate Trajectory
trajectory = TrajectoryGenerator(Tse_i,Tsc_i,Tsc_f,Tce_g,Tce_s,k,dt);

% Control Gains
Kp = 1*eye(6);
Ki = 0.001*eye(6);
control = [true true true];