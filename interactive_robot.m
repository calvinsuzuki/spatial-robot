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
lastPosition = PositionHandle([0, 0]);
set(figHandle, 'WindowButtonDownFcn', @(~, ~) onMouseClick(robot, figHandle, lastPosition));

function onMouseClick(robot, figHandle, lastPositionHandle)
    % Get clicked position
    point = get(gca, 'CurrentPoint');
    x = point(1, 1) - 400;
    y = point(1, 2) - 300;
    last_p = lastPositionHandle.Position;
    disp(lastPositionHandle.Position);
    
    [q1, q2] = inverseKinematics(x, y, robot.L1, robot.L2);
    
    time = 0:0.001:1;
    % setpoint = ones(length(time),2) .* [q1 q2];
    setpoint = lastPositionHandle.generateRamp([q1 q2], length(time));
        [state_history, ~] = planarPID(time, setpoint, [last_p(1) last_p(2) 0 0 0 0], [200 30 40]);
    % Save the state history for animation
    save('state_history.mat', 'state_history');
    save('setpoint.mat', 'setpoint');
    for j = 1:50:length(time)
        cla;
        image('CData', imread('aerotech_bkg.jpg'), 'XData', [0 800], 'YData', [0 600]);
        robot.plotRobot(state_history(j,1), state_history(j,2), figHandle, true);
        pause(0.001);
    end
    
    % Update lastPosition
    lastPositionHandle.Position = [state_history(end,1), state_history(end,2)];
end