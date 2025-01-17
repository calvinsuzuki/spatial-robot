classdef PositionHandle < handle
    properties
        Position
    end
    
    methods
        function obj = PositionHandle(initialPosition)
            obj.Position = initialPosition;
        end
        
        function ramp = generateRamp(obj, newPosition, steps)
            fprintf('Interp from %s to %s in %d steps\n', mat2str(obj.Position), mat2str(newPosition), steps);
            xRamp = linspace(obj.Position(1), newPosition(1), steps);
            yRamp = linspace(obj.Position(2), newPosition(2), steps);
            ramp = [xRamp', yRamp'];
        end
    end
end