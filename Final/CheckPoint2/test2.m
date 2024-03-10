clear 
clc



Tz = @(x,y,z,phi) [cos(phi)  -sin(phi) 0 x;
                  sin(phi)   cos(phi) 0 y;
                  0          0        1 z;
                  0          0        0 1];

Ty = @(x,y,z,phi) [cos(phi)  0 sin(phi) x;
                  0          1 0        y;
                  -sin(phi)  0 cos(phi) z;
                  0          0        0 1]

Tse_i = [1.0000    0         0    0.3334;
         0    1.0000         0         0;
         0         0    1.0000    0.7839;
         0         0         0    1.0000];

Tsc_i = Tz( 1,  0,  0,     0);
Tsc_f = Tz( 0, -1,  0, -pi/2);

Tce_g =  Ty(0,0,0,pi);
Tce_s =  Ty(0,0,0.25,pi)
k = 1;

trajectory = TrajectoryGenerator(Tse_i,Tsc_i,Tsc_f,Tce_g,Tce_s,k);

writematrix(trajectory,"trajectory.csv")

plot3(trajectory(:,10),trajectory(:,11),trajectory(:,12))

close
fig2 = figure(2);
title("Robot Arm Configuration")
axis([-1 1 -1 1 -1 1])
pbaspect([1 1 1])
daspect([1 1 1])
grid on
view(3)
x = trajectory(:,10);
y = trajectory(:,11);
z = trajectory(:,12);

xmax = 1;
ymax = 1;
zmax = 1;
h = gca;

[SX,SY] = meshgrid(linspace(-2,2,100),linspace(-2,2,100));
SZ = zeros(100,100);

a
a
a
axis


for i = 1:length(trajectory(:,1))
cla
hold on
plot3(x(1:i),y(1:i),z(1:i))
scatter3(x(i),y(i),z(i),'filled')
plot3(h.XLim, [0 0], [0 0],'k')
plot3([0, 0], h.YLim, [0 0],'k');
plot3([0, 0], [0 0], [0,h.ZLim(2)],'k');
xlim([x(i)-xmax,x(i)+xmax])
ylim([y(i)-ymax,y(i)+ymax])
zlim([z(i)-zmax,z(i)+zmax])
surf(SX,SY,SZ)
colormap summer
shading interp
hold off
view(50,20)
drawnow
pause(0.1)
end


