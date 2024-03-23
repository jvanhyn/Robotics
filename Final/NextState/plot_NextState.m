fig1 = figure(1);
title("Robot Spacial Configuration")
grid on
xmax = 1;
ymax = 1;
for i = 1:N
cla
hold on
subtitle("t = " + t(i))
plot(q(2,1:i),q(3,1:i))
quiver(q(2,i), q(3,i), cos(q(1,i)), sin(q(1,i)),0.1,"LineWidth",2);
scatter(q(2,i),q(3,i),"filled");
xlim([q(2,i)-xmax,q(2,i)+xmax])
ylim([q(3,i)-ymax,q(3,i)+ymax])
scatter(0,0,'k',"filled");
yline(0)
xline(0)
pbaspect([1 1 1])
hold off
drawnow
pause(0.01)

end