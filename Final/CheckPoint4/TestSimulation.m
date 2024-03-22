close all; clear; clc;
%#ok<*NASGU>

figfile = pwd + "/Figures/";
csvfile = pwd + "/csv/";

run best
run SimulateRobot
run PlotSimulation
writematrix(Q, csvfile + "robotmotion1.csv")


run overshoot
run SimulateRobot
run PlotSimulation
writematrix(Q, csvfile + "robotmotion2.csv")

run newtask
run SimulateRobot
run PlotSimulation
writematrix(Q, csvfile + "robotmotion3.csv")

figure(1)
saveas(gcf,figfile+"fig1.svg")
figure(2)
saveas(gcf,figfile+"fig2.svg")
figure(3)
saveas(gcf,figfile+"fig3.svg")

