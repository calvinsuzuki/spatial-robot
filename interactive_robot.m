% Interactive script
clear; clc; close all;

% Create robot instance
screenW = 800; screenH = 600;
robot = Planar2DOFRobot(180, 120, screenW/2, screenH/2);

figHandle = figure;
robot.plotRobot(0, 0, figHandle, true);

% Set up interactive screen
image('CData', imread('aerotech_bkg.jpg'), 'XData', [0 screenW], 'YData', [0 screenH]);
hold on;
axis([0 screenW 0 screenH]);
set(gca, 'YDir', 'reverse');
title('Click to Move the Robot End-Effector');
set(gca, 'XTick', [], 'YTick', []);
set(figHandle, 'WindowButtonDownFcn', @(~, ~) onMouseClick(robot, figHandle));

function onMouseClick(robot, figHandle)
    % Get clicked position
    point = get(gca, 'CurrentPoint');
    x = point(1, 1) - 400;
    y = point(1, 2) - 300;

    [q1, q2] = inverseKinematics(x, y, robot.L1, robot.L2);
    
    time = 0:0.001:6;
    setpoint = ones(length(time),2) .* [q1 q2];
    [state_history, ~] = planarPID(time, setpoint, [0 0 0 0 0 0], [60 30 40]);
    for i = 1:80:length(time)
        cla;
        image('CData', imread('aerotech_bkg.jpg'), 'XData', [0 800], 'YData', [0 600]);
        robot.plotRobot(state_history(i,1), state_history(i,2), figHandle, true);
        pause(0.001);
    end
end