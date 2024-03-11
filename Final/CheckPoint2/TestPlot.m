%% Plot the Trajectory
fig1 = figure(1);
title("Trajectory Overview")
plot3(trajectory(:,10),trajectory(:,11),trajectory(:,12))

fig2 = figure(2);
title("Robot Trajectory")
axis([-1 1 -1 1 -1 1])
pbaspect([1 1 1])
daspect([1 1 1])
grid on
view(3)
x = trajectory(:,10);
y = trajectory(:,11);
z = trajectory(:,12);

xmax = 1.5;
ymax = 1.5;
zmax = 1.5;
h = gca;

SZ = zeros(11,11);

for i = 1:length(trajectory(:,1))
cla(h)
hold on
plot3(h,x(1:i),y(1:i),z(1:i),'g')
scatter3(h,x(i),y(i),z(i),'filled')
xlim(h,[x(i)-xmax,x(i)+xmax])
ylim(h,[y(i)-ymax,y(i)+ymax])
zlim(h,[z(i)-zmax,z(i)+zmax])
[SX,SY] = meshgrid(linspace(-2*xmax,2*xmax,11),linspace(-2*ymax,2*ymax,11));
s = surf(h,SX,SY,SZ);
g = [0.7,0.7,0.7];
s.EdgeColor = g;
s.FaceColor = 'w';
plot3(h,h.XLim, [0 0], [0 0],'Color',g,'LineWidth',2);
plot3(h,[0, 0], h.YLim, [0 0],'Color',g,'LineWidth',2);
plot3(h,[0, 0], [0 0], [0,h.ZLim(2)],'Color',g,'LineWidth',2)
axis off
hold off
fig2.Color = 'w';
view(50,20)
drawnow
pause(0.01)
end

