function new_robot_places = update_robot_places(obj, transition, old_robot_places, robots_to_update)
    %
    new_robot_places = old_robot_places;
    trans_index = obj.find_transition_index(transition);
    [input_place_indices, col, input_val] = find(obj.input_arcs(:,trans_index));
    nInputPlaces = size(input_place_indices, 1);
    nInputRobots = 0;
    for pi_index = 1:nInputPlaces
        input_place_name = obj.places(input_place_indices(pi_index));
        if ~isempty(find(obj.robot_places == input_place_name))
            %Is a robot place, will remove a robot token
            nInputRobots = nInputRobots + input_val(pi_index);
        end
    end
    nRobotsInvolved = size(robots_to_update, 2);
    
    if (nInputRobots ~= nRobotsInvolved)
        error("The number of robots affected by transition is not the same number of robots given as input");
    end
    
    [row, output_place_indices, val] = find(obj.output_arcs(trans_index, :));
    nOutputPlaces = size(output_place_indices, 2);
    
    for ri_index = 1:nRobotsInvolved
        r_index = robots_to_update(ri_index);
        old_place_index = old_robot_places(r_index);
        if ~isempty(find(input_place_indices == old_place_index))
            for o_index = 1:nOutputPlaces
                output_place_name = obj.places(output_place_indices(o_index));
                if ~isempty(find(obj.robot_places == output_place_name))
                    new_robot_places(r_index) = output_place_indices(o_index);
                    break
                end
            end
        end
    end
    
end