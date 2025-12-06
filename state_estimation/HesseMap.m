classdef HesseMap < handle
    % HesseMap - Map representation using lines in Hesse normal form
    %
    % This class manages a 2D map composed of line segments represented
    % in Hesse normal form: (alpha, d), where:
    %   - alpha: angle of the line's normal vector [rad]
    %   - d: perpendicular distance from origin to line [m]
    %
    % Usage:
    %   map = HesseMap();
    %   map.addLine(alpha, d);
    %   lines = map.getLines();
    %   map.plot();

    properties
        lines       % Lines in Hesse form [alpha, d] (Nx2)
        num_lines   % Number of lines in the Hesse map
    end

    methods
        function obj = HesseMap()
            % HesseMap Constructor
            %
            % Creates an empty map.

            obj.lines = [];
            obj.num_lines = 0;
        end

        function addLine(obj, alpha, d)
            % addLine - Add a line in Hesse normal form
            %
            % Args:
            %   alpha: Angle of line normal vector [rad]
            %   d: Perpendicular distance from origin [m]

            obj.lines = [obj.lines; alpha, d];
            obj.num_lines = obj.num_lines + 1;
        end
    end
end
