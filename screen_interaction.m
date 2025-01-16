clear; clc; close all;

% Define fixed figure size
figureWidth = 800;  % Width in pixels
figureHeight = 600; % Height in pixels
figure('Position', [100, 100, figureWidth, figureHeight]);

% Display an image or blank screen
imagesc(zeros(figureHeight, figureWidth)); colormap white; axis image;

% Fix the axes limits and keep them constant
axis on;
axis([0 figureWidth 0 figureHeight]);
set(gca, 'YDir', 'reverse');
title('Click on the screen to see the animated circle follow the clicks.');

circleRadius = 20;
theta = linspace(0, 2*pi, 50);
xCircle = circleRadius * cos(theta); 
yCircle = circleRadius * sin(theta); 
animatedCircle = patch(xCircle, yCircle, 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'none');

% Start at the center of the screen
last_point = [figureWidth / 2, figureHeight / 2]; 

% Store variables in the figure's appdata
setappdata(gcf, 'animatedCircle', animatedCircle); % Store the patch object
setappdata(gcf, 'xCircle', xCircle); % Store circle X-coordinates
setappdata(gcf, 'yCircle', yCircle); % Store circle Y-coordinates
setappdata(gcf, 'last_point', last_point); % Store the last point
setappdata(gcf, 'isAnimating', false); % Animation status flag
set(animatedCircle, 'XData', last_point(1) + xCircle, 'YData', last_point(2) + yCircle)

% Callback function for mouse clicks
set(gcf, 'WindowButtonDownFcn', @getClick);

% Callback function definition
function getClick(~, ~)
    % Ignore new clicks until animation is complete
    if getData('isAnimating')
        return; 
    end
    
    % Get point
    point = get(gca, 'CurrentPoint');
    x = point(1, 1); y = point(1, 2);
    
    % Retrieve stored variables from appdata
    animatedCircle = getData('animatedCircle');
    xCircle = getData('xCircle');
    yCircle = getData('yCircle');
    last_point = getData('last_point');
    
    % Block the callback and start animation
    setData('isAnimating', true);
    numSteps = 20;    
    for i = 1:numSteps
        coord_x = last_point(1) + (x - last_point(1)) * i / numSteps;
        coord_y = last_point(2) + (y - last_point(2)) * i / numSteps;
        set(animatedCircle, 'XData', coord_x + xCircle, 'YData', coord_y + yCircle);
        pause(0.005);
    end
    
    % Update last point and unflag animation
    setData('last_point', [x y]);
    setData('isAnimating', false);
end

function data = getData(name)
    data = getappdata(gcf, name);
end

function setData(name, data)
    setappdata(gcf, name, data);
end