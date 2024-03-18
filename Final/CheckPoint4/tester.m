close 

figure()
h = gca
hold on
plot3(h,trajectory(:,10),trajectory(:,11),trajectory(:,12),'LineWidth',2)
plot3(h,x,y,z,'go','LineWidth',3)
hold off