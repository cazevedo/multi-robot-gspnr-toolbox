function new_robot_places = update_robot_places(obj, transition, old_robot_places, robots_to_update)
    %
    robots_available = robots_to_update;
    
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
    
    output_robot_place_indices = [];
    output_robot_place_weights = [];
    
    %Clearing from list all of nonrobot places
    for o_index = 1:nOutputPlaces
        output_place_name = obj.places(output_place_indices(o_index));
        output_place_index = output_place_indices(o_index);
        output_place_weight = val(o_index);
        
        if ~isempty(find(obj.robot_places == output_place_name))
            output_robot_place_indices = cat(2, output_robot_place_indices, output_place_index);
            output_robot_place_weights = cat(2, output_robot_place_weights, output_place_weight);
        end
    end
    
    nNonRobotOutputPlaces = size(output_robot_place_indices, 2);
    
    for o_index = 1:nNonRobotOutputPlaces
        output_place_index = output_robot_place_indices(o_index);
        output_place_weights = output_robot_place_weights(o_index);
        output_place_type_index = obj.place_types(output_place_index);
        nRobotsForOutputPlace = output_place_weights;
        for no_index = 1:nRobotsForOutputPlace
            nRobotsAvailable = size(robots_available, 2);
            for nr_index = 1:nRobotsAvailable
                old_place_index = old_robot_places(robots_available(nr_index));
                old_place_type_index = obj.place_types(old_place_index);
                if old_place_type_index == output_place_type_index
                    new_robot_places(robots_available(nr_index)) = output_place_index;
                    robots_available(nr_index) = [];
                    break
                end
            end
        end
    end
            
        
    
%     output_types = unique(obj.type_list(obj.place_types(output_place_indices)));
%     nTypesInvolved = size(output_types, 2);
    
%     for ri_index = 1:nRobotsInvolved
%         r_index = robots_to_update(ri_index);
%         old_place_index = old_robot_places(r_index);
%         robot_type = obj.place_types(old_place_index);
%         if ~isempty(find(input_place_indices == old_place_index))
%             for o_index = 1:nOutputPlaces
%                 output_place_name = obj.places(output_place_indices(o_index));
%                 if ~isempty(find(obj.robot_places == output_place_name))
%                     new_robot_places(r_index) = output_place_indices(o_index);
%                     break
%                 end
%             end
%         end
%     end
    
end