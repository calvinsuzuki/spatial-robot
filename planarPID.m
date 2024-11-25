function [state_history,control_history] = planarPID(time, setpoint, initial_state, PID)
%planarPID Get PID response for a basic planar robot
%   Detailed explanation goes here

if length(time) ~= length(setpoint)
    warning('Mismatch on time and setpoint lenghts!')
    state_history = 0;
    control_history = 0;
    return
end

% Arguments
q = initial_state(1:2);
qd = initial_state(3:4);
tau = initial_state(5:6);
Kp = PID(1); Ki = PID(2); Kd = PID(3);

% Sampling time
dt = (time(end) - time(1)) / (length(time)-1);

% Aux variables (improve efficiency)
error_prev = [0 0]; integral = [0 0];
control_history = zeros(length(time), 2);
state_history = zeros(length(time), 6);

for i = 1:length(time)
    % Robot response
    qd = qd + FDab(planar(2), q, qd, tau)' * dt;
    q = q + qd * dt;    
    
    % Controller output
    error = setpoint(i,:) - q;
    integral = integral + error * dt;
    derivative = (error - error_prev) / dt;
    tau = Kp * error + Ki * integral + Kd * derivative;
    
    % Save
    control_history(i,:) = tau;
    state_history(i,:) = [q qd tau];
    
    % Update for next iteration
    error_prev = error;
end
return
end