classdef ExecutableGSPNR < GSPNR
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = private)
        place_actions = struct();
        policy;
    end
    
    methods
        function execGSPNR = ExecutableGSPNR()
            execGSPNR = execGSPNR@GSPNR();
        end
        
        function import_nonExecutable(execGSPNR, GSPNR, action_map)
            %Copy properties of non-executable instance;
            copy_gspn = copy(GSPNR);
            execGSPNR.places = copy_gspn.places;
            execGSPNR.transitions = copy_gspn.transitions;
            execGSPNR.type_transitions = copy_gspn.type_transitions;
            execGSPNR.rate_transitions = copy_gspn.rate_transitions;
            execGSPNR.input_arcs = copy_gspn.input_arcs;
            execGSPNR.output_arcs = copy_gspn.output_arcs;
            execGSPNR.arcs = copy_gspn.arcs;
            execGSPNR.initial_marking = copy_gspn.initial_marking;
            execGSPNR.current_marking = copy_gspn.current_marking;
            execGSPNR.place_rewards = copy_gspn.place_rewards;
            execGSPNR.transition_rewards = copy_gspn.transition_rewards;
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
                    execGSPNR.place_actions(place_index).place_name = place_name;
                    execGSPNR.place_actions(place_index).action_name = action_map.(type).(place_name).action_name;
                    execGSPNR.place_actions(place_index).message = action_map.(type).(place_name).message;
                end
            end     
        end
        
        function set_policy(execGSPNR, markings, transition)
            
        end
        
    end
end

