clear; clc; close all;

robot = Planar2DOFRobot(2, 1.5);

% Target position for the end-effector
xTarget = 2.5;
yTarget = 1.0;

% Compute joint angles using inverse kinematics
[q1, q2] = inverseKinematics(xTarget, yTarget, robot.L1, robot.L2);

% Visualize the robot
figHandle = figure;
robot.plotRobot(q1, q2, figHandle);

function [q1, q2] = inverseKinematics(x, y, L1, L2)
    % Compute inverse kinematics for a 2-DOF planar robot
    r = sqrt(x^2 + y^2);
    if r > (L1 + L2) || r < abs(L1 - L2)
        error('Target point is outside the reachable workspace.');
    end

    % Angle for the second joint
    cosTheta2 = (x^2 + y^2 - L1^2 - L2^2) / (2 * L1 * L2);
    sinTheta2 = sqrt(1 - cosTheta2^2); % Assume positive elbow up solution
    theta2 = atan2(sinTheta2, cosTheta2);

    % Angle for the first joint
    k1 = L1 + L2 * cosTheta2;
    k2 = L2 * sinTheta2;
    theta1 = atan2(y, x) - atan2(k2, k1);

    q1 = theta1;
    q2 = theta2;
end