% Extract Trajectory xyz
x = trajectory(:,10);
y = trajectory(:,11);
z = trajectory(:,12);
xmax = 2;
ymax = 2;
zmax = 1;

% New Figure Window
fig2 = figure(2);
fig2.Color = 'w';

% New Axis
ax = gca;
axis(ax,[-1 1 -1 1 -1 1]);
pbaspect(ax,[1 1 1])
daspect(ax,[1 1 1])
title(ax,"Trajectory Ovewview")
view(3)

%
t = 0;

% Plot of the ground
SZ = zeros(11,11);
[SX,SY] = meshgrid(linspace(-2*xmax,2*xmax,11),linspace(-2*ymax,2*ymax,11));

for i = 1:k:length(trajectory(:,1))
cla(ax)
hold on

plot3(ax,x,y,z,'b','LineWidth',1)
plot3(ax,x(1:i),y(1:i),z(1:i),'g','LineWidth',3)
scatter3(ax,x(i),y(i),z(i),'r','filled')

xlim(ax,[x(i)-xmax,x(i)+xmax])
ylim(ax,[y(i)-ymax,y(i)+ymax])
zlim(ax,[z(i)-zmax,z(i)+zmax])

s = surf(ax,SX,SY,SZ);
g = [0.7,0.7,0.7];
s.EdgeColor = g;
s.FaceColor = 'w';

plot3(ax,ax.XLim, [0 0], [0 0],'Color',g,'LineWidth',2);
plot3(ax,[0, 0], ax.YLim, [0 0],'Color',g,'LineWidth',2);
plot3(ax,[0, 0], [0 0], [0,ax.ZLim(2)],'Color',g,'LineWidth',2)

axis off
hold off

view(50,20)
t = t + dt;
subtitle(ax,t)
drawnow
pause(dt)
end

