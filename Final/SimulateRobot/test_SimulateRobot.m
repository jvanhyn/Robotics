%% This script runs SimulateRobot appropiately and saves the results across several trials
% Clear Workspace
close all; clear; clc;

% Add paths to depended files
addpath(cd,"../mr/")
addpath(cd,"../Trajectory/")
addpath(cd,"../FeedbackControl/")
addpath(cd,"../NextState/")

% Save Folders
figfile = pwd + "/figs/";
csvfile = pwd + "/csv/";

disp("Simulating Robot...")

disp("-> 1")
% Best Performace
run(pwd + "/trials/best.m")
run SimulateRobot
run plot_SimulateRobot
writematrix(Q, csvfile + "best.csv")

disp("-> 2")
% Overshoot
run(pwd + "/trials/overshoot.m")
run SimulateRobot
run plot_SimulateRobot
writematrix(Q, csvfile + "overshoot.csv")

disp("-> 3")
% Newtask
run(pwd + "/trials/newtask.m")
run SimulateRobot
run plot_SimulateRobot
writematrix(Q, csvfile + "newtask.csv")

disp("Saving...")
% Save Figures
figure(1)
saveas(gcf,figfile+"fig1.svg")
figure(2)
saveas(gcf,figfile+"fig2.svg")
figure(3)
saveas(gcf,figfile+"fig3.svg")
close all
disp("Complete!")

