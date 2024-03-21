close all; clear; clc;
%#ok<*NASGU>

figfile = pwd + "/Figures/";
csvfile = pwd + "/csv/";

Kp = 0.5*eye(6); 
Ki = 0.000*eye(6); 
control = [true true true];

run SimulateRobot
writematrix(Q, csvfile + "robotmotion1.csv")


Kp = 1*eye(6);
Ki = 0.1*eye(6);
control = [true true true];

run SimulateRobot
writematrix(Q, csvfile + "robotmotion2.csv")


Kp = 1*eye(6);
Ki = 0.001*eye(6);
control = [true true true];

run SimulateRobot
writematrix(Q, csvfile + "robotmotion3.csv")

lims = 1e-2*[-1 1];

figure(1)
ylim(lims)
saveas(gcf,figfile+"fig1.svg")
figure(2)
ylim(lims)
saveas(gcf,figfile+"fig2.svg")
figure(3)
ylim(lims)
saveas(gcf,figfile+"fig3.svg")

