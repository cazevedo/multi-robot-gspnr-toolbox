function [fire, robots_involved, new_RobotFlags] = check_transition(obj, robot_index, transition, RobotPlaces, RobotFlags)
%CHECK_TRANSITION Used after action client for specific robot has returned, check if
%exponential transition modelling end of action can fire;

    robots_involved = [robot_index];
    new_RobotFlags = RobotFlags;
    
    trans_index = obj.find_transition_index(transition);
    [input_place_indices, ~, weights] = find(obj.input_arcs(:,trans_index));
    nInputPlaces = size(input_place_indices, 1);
    
    for index = 1:nInputPlaces
        input_place_index = input_place_indices(index);
        arc_weight = weights(index);
        input_place_name = obj.places(input_place_index);
        robot_place = RobotPlaces(robot_index);
        if robot_place == input_place_index
            arc_weight = arc_weight - 1;
        end
        done = true;
        robot_indices_in_input_place = find(RobotPlaces == input_place_index);
        robot_flags_in_input_place = RobotFlags(robot_indices_in_input_place);
        robot_indices_waiting_in_input_place = find(robot_flags_in_input_place == "WAIT");
        nRobotsWaiting = size(robot_indices_waiting_in_input_place, 2);
        if nRobotsWaiting >= arc_weight
            if ~isempty(robot_indices_waiting_in_input_place)
                chosen_robots = randsample(robot_indices_waiting_in_input_place, arc_weight);
                if isempty(chosen_robots)
                    chosen_robots = []
                end
            else
                chosen_robots = [];
            end
            robots_involved = cat(2, robots_involved, chosen_robots);
            continue;
        else
            done = false;
            break;
        end
    end
    if done
        fire = true;
        new_RobotFlags(robots_involved) = "FIN";
    else
        fire = false;
        new_RobotFlags(robot_index) = "WAIT";
    end  
    
end

