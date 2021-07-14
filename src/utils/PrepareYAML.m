function PrepareYAML(gspn, yaml_filepath)
%Given a GSPNR object, and with user's input, creates YAML file to fill out with the necessary information about the action servers and message types
    robot_type_struct = struct();

    [exp_trans, exp_trans_indices] = gspn.get_exponential_transitions();
    
    nEXPTrans = size(exp_trans, 2);
    
    action_places = [];
    
    for t_index = 1:nEXPTrans
        exp_trans_index = exp_trans_indices(t_index);
        input_places = gspn.input_arcs(:,exp_trans_index);
        input_place_indices = find(input_places);
        action_places = cat(2, action_places, input_place_indices');
    end
    action_places = unique(action_places);
    
    nActionPlaces = size(action_places, 2);
    
    for p_index = 1:nActionPlaces
        place_index = action_places(p_index);
        place_name = gspn.places(place_index);
        if ~startsWith(place_name, "r.")        
            robot_type_struct.(place_name).action_name = '';
            robot_type_struct.(place_name).message_type = '';
            robot_type_struct.(place_name).message_fields = '';
        end
    end
        
    WriteYaml(yaml_filepath, robot_type_struct);

end

