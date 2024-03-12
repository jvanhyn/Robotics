

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

% for i = 1:N
%     xyz(:,i) = T_0e(theta(:,i))*[0;0;0;1];
% end
% 
% close
% fig2 = figure(2);
% title("Robot Arm Configuration")
% axis([-1 1 -1 1 -1 1])
% pbaspect([1 1 1])
% daspect([1 1 1])
% view(3)
% 
% x = xyz(1,:);
% y = xyz(2,:);
% z = xyz(3,:);
% 
% xmax = 1;
% ymax = 1;
% zmax = 1;
% h = gca;
% 
% for i = 1:N
% cla
% hold on
% plot3(x(1:i),y(1:i),z(1:i))
% scatter3(x(i),y(i),z(i),'filled')
% plot3(h.XLim, [0 0], [0 0],'k')
% plot3([0, 0], h.YLim, [0 0],'k');
% plot3([0, 0], [0 0], h.ZLim,'k');
% xlim([x(i)-xmax,x(i)+xmax])
% ylim([y(i)-ymax,y(i)+ymax])
% zlim([z(i)-zmax,z(i)+zmax])
% hold off
% drawnow
% pause(0.01)
% end