classdef Planar2DOFRobot < handle
    properties
        L1 = 2;  % Length of the first link
        L2 = 1.5; % Length of the second link
        xBase = 0; % X-coordinate of the base of the robot
        yBase = 0; % Y-coordinate of the base of the robot
        Position = [0, 0]; % Current position of the end-effector
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
        
        function obj = plotRobot(obj, q1, q2, figHandle, holdFig)
            obj.Position = [q1, q2]; % Update the end-effector position
            % Calculate joint positions
            x0 = obj.xBase; y0 = obj.yBase;
            x1 = x0 + obj.L1 * cos(q1);
            y1 = y0 + obj.L1 * sin(q1);
            x2 = x1 + obj.L2 * cos(q1 + q2);
            y2 = y1 + obj.L2 * sin(q1 + q2);

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
            
            % Plot the background image
            image('CData', imread('aerotech_bkg.jpg'), 'XData', [0 800], 'YData', [0 600]);

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

            rotationMatrix = rz(q1 + q2);  % Use rotation matrix from Featherstone library
            rotatedCoords = rotationMatrix(1:2,1:2)' * [xC; yC];
            xC = rotatedCoords(1, :) + x2;
            yC = rotatedCoords(2, :) + y2;
            plot(xC, yC, 'k-', 'LineWidth', 2);
        end

        function response = ramp(obj, newPosition, velocity, sampling_time, PID)
            % Optimal trajectory planning
            for i = 1:2
                if obj.Position(i) - newPosition(i) > pi
                    newPosition(i) = newPosition(i) + 2 * pi;
                elseif obj.Position(i) - newPosition(i) < -pi
                    newPosition(i) = newPosition(i) - 2 * pi;
                end
            end

            % Define ramp time
            distance = norm(newPosition - obj.Position);
            ramp_time = distance / velocity;
            time = 0:sampling_time:ramp_time;
            steps = length(time);            

            xRamp = linspace(obj.Position(1), newPosition(1), steps);
            yRamp = linspace(obj.Position(2), newPosition(2), steps);
            setpoint = [xRamp', yRamp'];

            % Add 1 second of zero setpoint at the end
            steady_time = ramp_time:sampling_time:ramp_time+1;
            steady_steps = length(steady_time);
            xSteady = repmat(newPosition(1), steady_steps, 1);
            ySteady = repmat(newPosition(2), steady_steps, 1);
            setpoint = [setpoint; [xSteady, ySteady]];
            time = [time, steady_time];
            
            initial_state = [obj.Position(1) obj.Position(2) 0 0 0 0]; % Assume zero velocity and torque
            [state_history, ~] = planarPID(time, setpoint, initial_state, PID);

            response = state_history;
        end
        
        % function transition = sigmoid_transition(obj, start, final, duration)
        %     % Sigmoid transition between two points
        %     % start: initial point
        %     % final: final point
        %     % duration: duration of the transition
        %     t = linspace(0, 1, duration);
        %     transition = (final - start) * (1 - cos(pi * t)) / 2 + start;
        % end


        function [q1, q2] = ikine(obj, x, y)
            % Move x y to base frame
            x = x - obj.xBase;
            y = y - obj.yBase;

            % Compute inverse kinematics for a 2-DOF planar robot
            r = sqrt(x^2 + y^2);
            outerRadius = obj.L1 + obj.L2;
            innerRadius = abs(obj.L1 - obj.L2);
        
            if r > outerRadius
                fprintf('Warning: Target point is outside the reachable workspace.\n');
                x = x * outerRadius / r;
                y = y * outerRadius / r;
            elseif r < innerRadius
                fprintf('Warning: Target point is outside the reachable workspace.\n');
                x = x * innerRadius / r;
                y = y * innerRadius / r;
            end
        
            % Angle for the second joint
            cosTheta2 = (x^2 + y^2 - obj.L1^2 - obj.L2^2) / (2 * obj.L1 * obj.L2);
            if cosTheta2 > 1
                cosTheta2 = 1;
            elseif cosTheta2 < -1
                cosTheta2 = -1;
            end
        
            sinTheta2 = sqrt(1 - cosTheta2^2); % Assume positive elbow up solution
            theta2 = atan2(sinTheta2, cosTheta2);
        
            % Angle for the first joint
            k1 = obj.L1 + obj.L2 * cosTheta2;
            k2 = obj.L2 * sinTheta2;
            theta1 = atan2(y, x) - atan2(k2, k1);
        
            q1 = theta1;
            q2 = theta2;
        end

        function [vx, vy] = getEndEffectorState(obj, robot_state)
                q = robot_state(1:2);
                dq = robot_state(3:4);
                % ddq = robot_state(5:6);
                J = obj.getJacobian(q);
                vel_vec = J * dq' ./ 1000; % Convert to m/s
                vx = vel_vec(1); vy = vel_vec(2);
        end

        function J = getJacobian(obj, q)
            % Compute the Jacobian matrix for a 2-DOF planar robot
            q1 = q(1); q2 = q(2);
            J11 = -obj.L1 * sin(q1) - obj.L2 * sin(q1 + q2);
            J12 = -obj.L2 * sin(q1 + q2);
            J21 = obj.L1 * cos(q1) + obj.L2 * cos(q1 + q2);
            J22 = obj.L2 * cos(q1 + q2);
            J = [J11, J12; J21, J22];
        end
    end
end
