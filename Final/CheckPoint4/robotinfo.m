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

% Joint Limits (U,THETA)
joint_lims =  pi*[0.5 	0.7 	0.7 	0.7 	0.75 	0.75];