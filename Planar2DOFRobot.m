classdef Planar2DOFRobot
    properties
        L1 = 2;  % Length of the first link
        L2 = 1.5; % Length of the second link
    end
    
    methods
        function obj = Planar2DOFRobot(L1, L2)
            if nargin > 0
                obj.L1 = L1;
                obj.L2 = L2;
            end
        end
        
        function plotRobot(obj, q1, q2, figHandle)
            % Joint angles (in radians)
            theta1 = q1; theta2 = q2;

            % Calculate joint positions
            x0 = 0; y0 = 0; % Base of the robot
            x1 = obj.L1 * cos(theta1); % End of the first link
            y1 = obj.L1 * sin(theta1);
            x2 = x1 + obj.L2 * cos(theta1 + theta2); % End-effector position
            y2 = y1 + obj.L2 * sin(theta1 + theta2);

            % Plot the robot
            figure(figHandle);
            cla; hold on; axis equal;

            % Plot the reachable area (working volume)
            theta = linspace(0, 2*pi, 100);
            outerRadius = obj.L1 + obj.L2;
            innerRadius = abs(obj.L1 - obj.L2);
            xOuter = outerRadius * cos(theta);
            yOuter = outerRadius * sin(theta);
            xInner = innerRadius * cos(theta);
            yInner = innerRadius * sin(theta);
            fill([xOuter, fliplr(xInner)], [yOuter, fliplr(yInner)], 'y', 'FaceAlpha', 0.2, 'EdgeColor', 'none');

            % Plot the links
            plot([x0, x1], [y0, y1], 'b-', 'LineWidth', 2); % First link
            plot([x1, x2], [y1, y2], 'r-', 'LineWidth', 2); % Second link

            % Plot the joints and end-effector
            plot(x0, y0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k'); % Base joint
            plot(x1, y1, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k'); % First joint
            % plot(x2, y2, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'k'); % End-effector

            % Draw a "C" on the end-effector
            r = 0.15; % Radius of the "C"
            theta = linspace(pi/4, 2*pi - pi/4, 100); % Angle range for the "C"
            xC = r * cos(theta) + r; % X-coordinates of the "C" before rotation
            yC = r * sin(theta); % Y-coordinates of the "C" before rotation

            % Rotate the "C" along the end-effector angle
            rotationMatrix = [cos(theta1 + theta2), -sin(theta1 + theta2); 
                              sin(theta1 + theta2),  cos(theta1 + theta2)];
            rotatedCoords = rotationMatrix * [xC; yC];
            xC = rotatedCoords(1, :) + x2;
            yC = rotatedCoords(2, :) + y2;
            plot(xC, yC, 'k-', 'LineWidth', 2);

            % Annotate the plot
            xlabel('X-axis');
            ylabel('Y-axis');
            title('Planar 2-DOF Robot with a "C" on the End-Effector');
            legend('Link 1', 'Link 2', 'Location', 'Best');
        end
    end
end