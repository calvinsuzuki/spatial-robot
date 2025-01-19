clear; clc; close all;

% Set up interactive screen
screenW = 800; screenH = 600;
robot = Planar2DOFRobot(180, 120, screenW/2, screenH/2);
figHandle = figure;
hold on;
axis([0 screenW 0 screenH]);
set(gca, 'YDir', 'reverse');
title('Click to Move the Robot End-Effector');
set(gca, 'XTick', [], 'YTick', []);
robot.plotRobot(0, 0, figHandle, true);

% Set up mouse click event
set(figHandle, 'WindowButtonDownFcn', @(~, ~) onMouseClick(robot, figHandle));

function onMouseClick(robot, figHandle)
    % Get clicked position
    point = get(gca, 'CurrentPoint');
    x = point(1, 1); y = point(1, 2);
    [q1, q2] = robot.ikine(x, y);
    
    ramp_time = 1;
    sampling_time = 0.001;
    PID = [200 30 40];

    response = robot.ramp([q1, q2], ramp_time, sampling_time, PID);
    for j = 1:40:length(response)
        cla;
        robot.plotRobot(response(j,1), response(j,2), figHandle, true);
        robot.getEndEffectorState(response(j,:));
        pause(0.001);
    end
    robot.getEndEffectorState(response(end,:));
end