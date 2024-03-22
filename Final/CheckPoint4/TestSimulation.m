close all; clear; clc;
%#ok<*NASGU>

figfile = pwd + "/figs/";
csvfile = pwd + "/csv/";

run best
run SimulateRobot
run PlotSimulation
writematrix(Q, csvfile + "best.csv")


run overshoot
run SimulateRobot
run PlotSimulation
writematrix(Q, csvfile + "overshoot.csv")

run newtask
run SimulateRobot
run PlotSimulation
writematrix(Q, csvfile + "newtask.csv")

figure(1)
saveas(gcf,figfile+"fig1.svg")
figure(2)
saveas(gcf,figfile+"fig2.svg")
figure(3)
saveas(gcf,figfile+"fig3.svg")

