# MAE 204 Final Project: Winter 2024
# Jonathan Van Hyning and Ananya Thridandam

**If you do not have time to read this document, skip to the section on Checkpoint 4 and run the TestSimulation script**

Project Assignment: https://hades.mech.northwestern.edu/index.php/Mobile_Manipulation_Capstone_2023 
Resources: http://www.youbot-store.com/wiki/index.php/YouBot_Detailed_Specifications

The MATLAB code for this project is organized as follows:

The CheckPoint 1 folder contains the following scripts:
	-TrajectoryGenerator: used to generate the robot's desired trajectory

	-TrajectoryTest: tests the TrajectoryGenerator function
	-TrajectoryTestAnimation: a supplementary file to visualize the trajectory in MATLAB
	Note: the TrajectoryGenerator function is compatible with CoppeliaSim scene 8

The CheckPoint 2 folder contains the following scripts:
	-NextState: predicts how the robot will move in a small time step

	-NextTest: tests the NextState function
	-NextTestAnimation: a supplementary file to visualize the robot motion generated in NextTest in MATLAB

The CheckPoint 3 folder contains the following scripts:
	-FeebackControl: uses the desired trajectory to command joint velocities

	-FeedbackTest: tests the FeedbackControl function according to the checks in the assignment

The CheckPoint 4 folder contains the following scripts:
	-TestSimulation: parent script which calls and runs all other scripts in this directory
	-robotinfo: information regarding the robot specifications

	-best: contains initial conditions for and generates the desired trajectory for the best case control 	scenario
	-overshoot: contains initial conditions for and generates the desired trajectory for the overshoot control 	scenario
	-newtask: contains initial conditions for and generates the desired trajectory for the new task control 	scenario

	-SimulateRobot: simulates the mobile manipulation of a cube using the youBot by calling the FeedbackControl 	and NextState functions
	-PlotSimulation: plots the error generated from SimulateRobot.m