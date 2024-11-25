clear; clc; close all;

dt = 0.001; t_end = 10;
time = 0:dt:t_end;
q = [10 90] * pi/180;

transition_time = 2;
steady_time = 2;
t_slope = 0:dt:transition_time;

down_signal = zeros(length(0:dt:2), 2);
steady_signal = ones(length(0:dt:2), 2) .* q;
ramp_up = [t_slope; t_slope]' .* q / transition_time;
ramp_down = q-[t_slope; t_slope]' .* q / transition_time;

setpoint = [down_signal; ramp_up(2:end,:); steady_signal(2:end,:); ramp_down(2:end,:); down_signal(2:end,:)];

[state_history, control_history] = planarPID(time, setpoint, [0 0 0 0 0 0], [60 30 40]);

% Plot the results
figure;
plot(time, setpoint(:,1), 'k', 'LineWidth', 2); hold on;
plot(time, setpoint(:,2), 'k--', 'LineWidth', 2);
plot(time, state_history(:,1), 'r', 'LineWidth', 2); hold on;
plot(time, state_history(:,2), 'b', 'LineWidth', 2);

xlabel('Time (s)'); ylabel('Value');

legend('Input 1', 'Input 2', 'Joint 1', 'Joint 2');
title('PID Controller Simulation');

grid on;

showmotion(planar(2),time,state_history(:, 1:2)')