close all
clear
clc

run("/Users/jvanhyn/Documents/GitHub/Robotics/Final/setup.m")
load("/Users/jvanhyn/Documents/GitHub/Robotics/Final/setup.mat","T_0e");

du_max = 10;

t0 = 0;
tf = 2;
dt = 0.01;

N = cast((tf - t0)/dt,"uint8");

t = linspace(t0,tf,N);

gripClosed = false;

du = -10*[-0.5,1,1,-0.5]';
dtheta = [0,0,0,10,0]';

q0 = [0,0,0]';
u0 = [0,0,0,0]';
theta0 = [0,0,0,0,0]';

q = zeros(3,N);
u = zeros(4,N);
theta = zeros(5,N);

q(:,1) = q0;
u(:,1) = u0;
theta(:,1) = theta0;

Q(1,:) = [q(:,1);theta(:,1);u(:,1);gripClosed]';

for i = 1:N
    [q(:,i+1),theta(:,i+1),u(:,i+1)] = NextState(q(:,i),u(:,i),theta(:,i),du,dtheta,dt,du_max);
    Q(i+1,:) = [q(:,i+1);theta(:,i+1);u(:,i+1);gripClosed]';
end

writematrix(Q,'robotMotion.csv')

for i = 1:N
    xyz(:,i) = T_0e(theta(:,i))*[0;0;0;1];
end


close 
fig1 = figure(1);
title("Robot Spacial Configuration")
pbaspect([1 1 1])
daspect([1 1 1])

grid on

xmax = 1;
ymax = 1;
for i = 1:N
cla
hold on
subtitle(t(i))
plot(q(2,1:i),q(3,1:i))
quiver(q(2,i), q(3,i), cos(q(1,i)), sin(q(1,i)),0.1,"LineWidth",2);
scatter(q(2,i),q(3,i),"filled");
xlim([q(2,i)-xmax,q(2,i)+xmax])
ylim([q(3,i)-ymax,q(3,i)+ymax])
scatter(0,0,'k',"filled");
yline(0)
xline(0)
hold off
drawnow
pause(0.01)
end

close
fig2 = figure(2);
title("Robot Arm Configuration")
axis([-1 1 -1 1 -1 1])
pbaspect([1 1 1])
daspect([1 1 1])
view(3)

x = xyz(1,:);
y = xyz(2,:);
z = xyz(3,:);

xmax = 1;
ymax = 1;
zmax = 1;
h = gca;

for i = 1:N
cla
hold on
plot3(x(1:i),y(1:i),z(1:i))
scatter3(x(i),y(i),z(i),'filled')
plot3(h.XLim, [0 0], [0 0],'k')
plot3([0, 0], h.YLim, [0 0],'k');
plot3([0, 0], [0 0], h.ZLim,'k');
xlim([x(i)-xmax,x(i)+xmax])
ylim([y(i)-ymax,y(i)+ymax])
zlim([z(i)-zmax,z(i)+zmax])
hold off
drawnow
pause(0.01)
end


