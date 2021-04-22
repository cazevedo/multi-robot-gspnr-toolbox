function robots_involved = CheckRobotsInvolved(exec, transition, RobotPlaces)
%CHECKROBOTSINVOLVED Summary of this function goes here
%   Detailed explanation goes here
    robots_involved = [];
    trans_index = exec.find_transition_index(transition);
    [input_place_indices, col, val] = find(exec.input_arcs(:, trans_index));
    nInputPlaces = size(input_place_indices, 1);
    for pi_index = 1:nInputPlaces
        input_place_index = input_place_indices(pi_index);
        input_place_name = exec.places(input_place_index);
        arc_weight = val(pi_index);
        %Check if input place is a robot place
        if ~isempty(find(exec.robot_places == input_place_name))
            %If so, the number of robots involved is the arc weight
            %Check which robots are in this place
            robots_in_place = find(RobotPlaces == input_place_index);
            if size(robots_in_place, 2) == 1
                robots_involved = cat(2, robots_involved, robots_in_place);
            else
                robots_involved = cat(2, robots_involved, randsample(robots_in_place, arc_weight));
            end
        end
    end
end

