classdef ExecutableGSPNR < GSPNR
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = private)
        place_actions = struct();
        policy;
        ambiguity = false;
    end
    
    methods
        function execGSPNR = ExecutableGSPNR()
            execGSPNR = execGSPNR@GSPNR();
        end
        
        function import_nonExecutable(obj, GSPNR, action_map)
            %Copy properties of non-executable instance;
            copy_gspn = copy(GSPNR);
            obj.places = copy_gspn.places;
            obj.transitions = copy_gspn.transitions;
            obj.type_transitions = copy_gspn.type_transitions;
            obj.rate_transitions = copy_gspn.rate_transitions;
            obj.input_arcs = copy_gspn.input_arcs;
            obj.output_arcs = copy_gspn.output_arcs;
            obj.arcs = copy_gspn.arcs;
            obj.initial_marking = copy_gspn.initial_marking;
            obj.current_marking = copy_gspn.current_marking;
            obj.place_rewards = copy_gspn.place_rewards;
            obj.transition_rewards = copy_gspn.transition_rewards;
            %Fill out the action properties;
            robot_types = string(fieldnames(action_map));
            nTypes = size(robot_types, 1);
            for t_index = 1:nTypes
                type = robot_types(t_index);
                action_places = string(fieldnames(action_map.(type)));
                nActionPlaces = size(action_places, 1);
                for a_index = 1:nActionPlaces
                    place_name = action_places(a_index);
                    place_index = GSPNR.find_place_index(place_name);
                    obj.place_actions(place_index).place_name = place_name;
                    obj.place_actions(place_index).action_name = action_map.(type).(place_name).action_name;
                    obj.place_actions(place_index).message = action_map.(type).(place_name).message;
                end
            end     
        end
        
        function check_robot_ambiguity(obj)
            nTrans = size(obj.transitions, 2);
            for t_index = 1:nTrans
                [input_place_indices, col, val] = find(obj.input_arcs(:,t_index));
                if sum(val)>1
                    disp_string = "There is an input token/robot ambiguity involving transition - "+obj.transitions(t_index);
                    disp(disp_string);
                    ambiguity = true;
                end
                [row, output_place_indices, val] = find(obj.output_arcs(t_index, :));
                if sum(val)>1
                    disp_string = "There is an output token/robot ambiguity involving transition - "+obj.transitions(t_index);
                    disp(disp_string);
                    ambiguity = true;
                end
            end
        end
        
        function set_policy(obj, markings, transition)
            
        end
        
    end
end

