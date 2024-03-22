
% Helper Transformation Matrix Functions
Tx = @(x,y,z,phi) [1         0        0        x; 0         cos(phi) -sin(phi) y; 0         sin(phi) cos(phi) z; 0 0 0 1];
Ty = @(x,y,z,phi) [cos(phi)  0        sin(phi) x; 0         1        0         y; -sin(phi) 0        cos(phi) z; 0 0 0 1];
Tz = @(x,y,z,phi) [cos(phi) -sin(phi) 0        x; sin(phi)  cos(phi) 0         y; 0         0        1        z; 0 0 0 1];

%% Robot Info
% Reference Frames
% End-Effector      {e}
% Base              {0}
% Body              {b}
% Space             {s}

% Default Configurations
M0e =  [1 0 0 0.0330; 0 1 0 0; 0 0 1 0.6546; 0 0 0 1]; % default End-Effector configuration
Tb0 =  [1 0 0 0.1667; 0 1 0 0; 0 0 1 0.0026; 0 0 0 1]; % Base to Body frame transformation

% Manipulator screw axes
B1 = [0,  0,  1,  0,      0.0330,  0]';
B2 = [0, -1,  0, -0.5076,      0,  0]';
B3 = [0, -1,  0, -0.3526,      0,  0]';
B4 = [0, -1,  0, -0.2176,      0,  0]';
B5 = [0,  0,  1,       0,      0,  0]';
Blist = [B1,B2,B3,B4,B5];

% Chasis z-height
zsb = 0.0963;

% Joint Velocity Limmits (DU,DTHETA)
speed_max =  2*pi*[5250 	5250 	5250 	2850 	2800 	5250]/60;

%% Robot Initial conditions
q0 = [pi/6,-0.2,0]';                   % position of body (x,y,phi)
u0 = [0,0,0,0]';                       % wheel angles     (R^4)
theta0 = [0,pi/4,-pi/4,-pi/6,0]';      % joint angles     (R^5)

%% Generate Trajectory
% Trajectory initial conditions
qi = [0,0,0]';                  % position of body (x,y,phi)
ui = [0,0,0,0]';                       % wheel angles     (R^4)
thetai = [0,pi/4,-pi/4,-pi/2,0]';      % joint angles     (R^5)

% Trajectory initial conditions
Tsb_i = Tz(qi(2),qi(3),zsb,qi(1));       % Body to Spacial frame transformation
T0e_i = FKinBody(M0e,Blist,thetai);      % End-Effector to Base transformation

% Waypoints
Tse_i = Tsb_i * Tb0 * T0e_i;           % initial   configuration of the end-effector   (space frame)
Tce_s = Ty( 0, 0, .25, pi/2);          % standoff  configuration of the end-effector   (cube frame)
Tce_g = Ty( 0.01, 0,  0.02, pi/2);     % grasping  configuration of the end-effector   (cube frame)
Tsc_i = Tz( 1,  0,  0,  0);            % initial   configuration of the cube           (space frame)
Tsc_f = Tz( 0, -1,  0, -pi/2);         % final     configuration of the cube           (space frame)

% Resolution
dt = 0.01;                             % time step in seconds
k = 10;                                % frames per time step

% Generate Trajectory
trajectory = TrajectoryGenerator(Tse_i,Tsc_i,Tsc_f,Tce_g,Tce_s,k,dt);

% Control Gains
Kp = 1*eye(6); 
Ki = 1*eye(6); 
control = [true true true];