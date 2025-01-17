classdef Planar2DOFRobot
    properties
        L1 = 2;  % Length of the first link
        L2 = 1.5; % Length of the second link
        xBase = 0; % X-coordinate of the base of the robot
        yBase = 0; % Y-coordinate of the base of the robot
    end
    
    methods
        function obj = Planar2DOFRobot(L1, L2, xBase, yBase)
            if nargin > 0
                obj.L1 = L1;
                obj.L2 = L2;
            end
            if nargin > 2
                obj.xBase = xBase;
                obj.yBase = yBase;
            end
        end
        
        function plotRobot(obj, q1, q2, figHandle, holdFig)
            % Joint angles (in radians)
            theta1 = q1; theta2 = q2;

            % Calculate joint positions
            x0 = obj.xBase; y0 = obj.yBase; % Base of the robot
            x1 = x0 + obj.L1 * cos(theta1); % End of the first link
            y1 = y0 + obj.L1 * sin(theta1);
            x2 = x1 + obj.L2 * cos(theta1 + theta2); % End-effector position
            y2 = y1 + obj.L2 * sin(theta1 + theta2);

            if ~holdFig
                figure(figHandle);
                hold on; axis equal;

                % Plot the reachable area (working volume) using circle outlines
                theta = linspace(0, 2*pi, 100);
                outerRadius = obj.L1 + obj.L2;
                innerRadius = abs(obj.L1 - obj.L2);
                xOuter = outerRadius * cos(theta) + obj.xBase;
                yOuter = outerRadius * sin(theta) + obj.yBase;
                xInner = innerRadius * cos(theta) + obj.xBase;
                yInner = innerRadius * sin(theta) + obj.yBase;
                plot(xOuter, yOuter, 'y--', 'LineWidth', 1.5); % Outer circle
                plot(xInner, yInner, 'y--', 'LineWidth', 1.5); % Inner circle
            end

            % Plot the links
            plot([x0, x1], [y0, y1], 'b-', 'LineWidth', 2); % First link
            plot([x1, x2], [y1, y2], 'r-', 'LineWidth', 2); % Second link

            % Plot the joints and end-effector
            plot(x0, y0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k'); % Base joint
            plot(x1, y1, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k'); % First joint

            % Draw a "C" on the end-effector
            r = 15; % Radius of the "C"
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
        end
    end
end
