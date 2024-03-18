close all; clear; clc;

addpath(cd,'../mr')
addpath(cd,'../CheckPoint1')
addpath(cd,'../CheckPoint2')
addpath(cd,'../CheckPoint3')

%%
% Rotation matrix helper functions
Tz = @(x,y,z,phi) [cos(phi) -sin(phi) 0 x;
                  sin(phi)   cos(phi) 0 y;
                  0          0        1 z;
                  0          0        0 1];

Ty = @(x,y,z,phi) [cos(phi)  0 sin(phi) x;
                   0         1      0   y;
                   -sin(phi) 0 cos(phi) z;
                   0         0      0   1];

%% Robot Info
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

%% Initial conditions
q0 = [0,0,0]';                      % position of chasis         
u0 = [0,0,0,0]';                    % wheel angles
theta0 = [0,pi/4,-pi/4,-pi/2,0]';              % manipulator arm angles

Tsb = [cos(q0(1)) -sin(q0(1)) 0 q0(2); sin(q0(1))  cos(q0(1)) 0 q0(3); 0 0 1 0.0963; 0 0 0 1]; % Body to Spacial frame transformation
T0e = FKinBody(M0e,Blist,theta0); % End-effector to manipulator base transformation
Tse = Tsb * Tb0 * T0e;

Tse_i = Tse;
Tsc_i = Tz( 1,  0,  0,     0);     % initial configuration of the cube
Tsc_f = Tz( 0, -1,  0,     -pi/2);     % final configuration of the cube

Tce_g = Ty( .01, 0,  0.02, pi/2);        % grasp config of the ee wrt {c}
Tce_s = Ty( 0, 0, .3,  pi/2);        % standoff config of the ee wrt {c}

% %% Trajectory Generation
k = 10;
dt = 0.01;
speed_max = 200;
trajectory = TrajectoryGenerator(Tse_i,Tsc_i,Tsc_f,Tce_g,Tce_s,k);

%% Control Gains
Kp = eye(6);
Ki = zeros(6);

%% Integration Variables
q = q0;
u = u0;
theta = theta0;

%% Running The Simulation
N = length(trajectory(:,1));
Q = zeros(N,13);

Xd = Tse_i;
for i = 1:N-1
T0e = FKinBody(M0e,Blist,theta);
Tsb = [cos(q(1)) -sin(q(1)) 0 q(2); sin(q(1))  cos(q(1)) 0 q(3); 0 0 1 0.0963; 0 0 0 1]; % Body to Spacial frame transformation
Tse = Tsb * Tb0 * T0e;
x(i) = Tse(1,4);
y(i) = Tse(2,4);
z(i) = Tse(3,4);
Xd = [trajectory(i,1:3),trajectory(i,10);trajectory(i,4:6),trajectory(i,11);trajectory(i,7:9),trajectory(i,12);0,0,0,1];
Xd_n = [trajectory(i+1,1:3),trajectory(i+1,10);trajectory(i+1,4:6),trajectory(i+1,11);trajectory(i+1,7:9),trajectory(i+1,12);0,0,0,1];
[Vb(:,i),du(:,i),dtheta(:,i)] = FeedbackControl(q,theta,Tse,Xd,Xd_n,Kp,Ki,dt);
du(:,i);
Vs(:,i) = Adjoint(Tse)*Vb(:,i);
[q,theta,u] = NextState(q,u,theta,du(:,i),dtheta(:,i),dt,speed_max);

Q(i,:) = [q;theta;u;trajectory(i,13)]'; % store values of Q, the current configuration of the robot (
end

writematrix(Q,'robotmotion.csv')



% xmax = 1;
% ymax = 1;
% zmax = 1;
% figure()
% h = gca;
% for i = 1:N-1
% cla(h)
% hold on
% plot3(h,trajectory(:,10),trajectory(:,11),trajectory(:,12),'LineWidth',2)
% scatter3(h,trajectory(i,10),trajectory(i,11),trajectory(i,12),'filled')
% plot3(h,x(1:i),y(1:i),z(1:i),'g','LineWidth',2)
% scatter3(h,x(i),y(i),z(i),'filled')
% xlim(h,[x(i)-xmax,x(i)+xmax])
% ylim(h,[y(i)-ymax,y(i)+ymax])
% zlim(h,[z(i)-zmax,z(i)+zmax])
% % [SX,SY] = meshgrid(linspace(-2*xmax,2*xmax,11),linspace(-2*ymax,2*ymax,11));
% % s = surf(h,SX,SY,SZ);
% % g = [0.7,0.7,0.7];
% % s.EdgeColor = g;
% % s.FaceColor = 'w';
% % plot3(h,h.XLim, [0 0], [0 0],'Color',g,'LineWidth',2);
% % plot3(h,[0, 0], h.YLim, [0 0],'Color',g,'LineWidth',2);
% % plot3(h,[0, 0], [0 0], [0,h.ZLim(2)],'Color',g,'LineWidth',2)
% hold off
% fig2.Color = 'w';
% view(3)
% drawnow
% pause(0.01)
%end