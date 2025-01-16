clear; clc; close all;

robot = Planar2DOFRobot(2, 1.5);

% Define the number of frames for the animation
numFrames = 100;

% Define the range of motion for the joints
theta1_start = 0;
theta1_end = pi/2;
theta2_start = 0;
theta2_end = pi/2;

% Generate the joint angles for each frame
theta1 = linspace(theta1_start, theta1_end, numFrames);
theta2 = linspace(theta2_start, theta2_end, numFrames);

% Create a figure for the animation
figHandle = figure;
set(figHandle, 'Position', [100, 100, 800, 600]);
axis([-0 3.5 0 3.5]); % Set the axis limits

% Loop through each frame and update the robot's joint angles
for i = 1:numFrames
    robot.plotRobot(theta1(i), theta2(i), figHandle);
    pause(0.05); % Pause to control the speed of the animation
end
