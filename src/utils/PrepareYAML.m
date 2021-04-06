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
    
    nRobots = input("Input the amount of different robot types\n");
    robot_types_list = [string.empty];
    
    for r_index = 1:nRobots
        robot_types_list(r_index) = input("\nName of robot type - ", 's');
    end
    
    for p_index = 1:nActionPlaces
        place_index = action_places(p_index);
        place_name = gspn.places(place_index);
        prompt_string = "\nTo which robot type does "+place_name+" pertain? ";
        robot_type = input(prompt_string, 's');
        
        if isempty(find(robot_types_list == robot_type))
            error('Please input a robot type previously declared');
        end
        
        robot_type_struct.(robot_type).(place_name).action_name = '';
        robot_type_struct.(robot_type).(place_name).message_type = '';
        robot_type_struct.(robot_type).(place_name).message_fields = '';        
    end
    
    more = "NOT_DONE";
    
    while (more ~= "DONE")
        more = input("Are there any places not previously mentioned that are action places?\n If not input 'DONE', if so, input the place name\n",'s');
        
        if more == "DONE"
            break
        end
        
        if isempty(find(gspn.places == more))
            error('Please input a valid place name');
        end
        prompt_string = "\nTo which robot type does "+more+" pertain? ";
        robot_type = input(prompt_string, 's');
        
        if isempty(find(robot_types_list == robot_type))
            error('Please input a robot type previously declared');
        end
        
        robot_type_struct.(robot_type).(more).action_name = '';
        robot_type_struct.(robot_type).(more).message_type = '';
        robot_type_struct.(robot_type).(more).message_fields = '';
        
    end
        
    WriteYaml(yaml_filepath, robot_type_struct);

end

