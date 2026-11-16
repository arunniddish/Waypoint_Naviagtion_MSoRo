classdef world_decision

    properties (Access = public)
        target_position;
        stop_radius;
        pixel_to_cm;
        threshold_deviation;
    end

    properties (Access = private)
        decision
    end

    properties(Constant)
        decision
    end

    methods

        function obj = world_decision(params)
            obj.target_position = params.target_position;
            obj.stop_radius = params.stop_radius;
            obj.pixel_to_cm = params.pixel_to_cm;
        end

        function stats = goal_check(obj, curr_robo_position)
            X = [curr_robo_position/obj.pixel_to_cm ; obj.target_position/obj.pixel_to_cm];
            d = pdist(X,'euclidean');

            if d <= obj.stop_radius
                stats = true;
            else
                stats = false;
            end
        end

        function stats = path_follow_check(obj, curr_robo_position, simulation_robo_position)
            X = [curr_robo_position/obj.pixel_to_cm ; simulation_robo_position/obj.pixel_to_cm];
            d = pdist(X, 'euclidean');

            if d <= obj.threshold_deviation
                stats = true;
            else
                stats = false;
            end
        end

    end % Methods
end % Class