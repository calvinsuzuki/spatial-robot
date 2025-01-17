function [q1, q2] = inverseKinematics(x, y, L1, L2)
    % Compute inverse kinematics for a 2-DOF planar robot
    r = sqrt(x^2 + y^2);
    outerRadius = L1 + L2;
    innerRadius = abs(L1 - L2);

    if r > outerRadius
        fprintf('Warning: Target point is outside the reachable workspace.');
        x = x * outerRadius / r;
        y = y * outerRadius / r;
    elseif r < innerRadius
        fprintf('Warning: Target point is outside the reachable workspace.');
        x = x * innerRadius / r;
        y = y * innerRadius / r;
    end

    % Angle for the second joint
    cosTheta2 = (x^2 + y^2 - L1^2 - L2^2) / (2 * L1 * L2);
    if cosTheta2 > 1
        cosTheta2 = 1;
    elseif cosTheta2 < -1
        cosTheta2 = -1;
    end

    sinTheta2 = sqrt(1 - cosTheta2^2); % Assume positive elbow up solution
    theta2 = atan2(sinTheta2, cosTheta2);

    % Angle for the first joint
    k1 = L1 + L2 * cosTheta2;
    k2 = L2 * sinTheta2;
    theta1 = atan2(y, x) - atan2(k2, k1);

    q1 = theta1;
    q2 = theta2;
end