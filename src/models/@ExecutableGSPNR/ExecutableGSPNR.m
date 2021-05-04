classdef ExecutableGSPNR < GSPNR
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = private)
        unique_ROS_package_dependencies = [string.empty];
        nROSDependencies = 0;
        
        place_actions = struct();

        policy = struct();
        empty_policy = true;

        ambiguity = false;
        robot_conservation = true;
        
        robot_list = [string.empty];
        robot_initial_locations = [];
        nRobots = 0;
        
        robot_places = [string.empty];
        
        interface_action_servers = [string.empty];
        
        simple_exp_transitions = [];
        simple_exp_transition_flags = [];

    end
    
    methods
        function execGSPNR = ExecutableGSPNR()
            execGSPNR = execGSPNR@GSPNR();
        end
        function initialize(obj, GSPNR, YAML_filepath, action_map)
            %If YAML_filepath input is not empty, function will disregard
            %completely the third input "action_map";
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
            %Fill out the action properties, either directly from
            %action_map structure, and if input YAML filepath is not empty,
            %load action_map from file;
            if ~isempty(YAML_filepath)
                action_map = ReadfromYAML(YAML_filepath);
            end 
            action_places = string(fieldnames(action_map));
            nActionPlaces = size(action_places, 1);
            for a_index = 1:nActionPlaces
                place_name = action_places(a_index);
                place_index = GSPNR.find_place_index(place_name);
                obj.place_actions(place_index).place_name = place_name;
                obj.place_actions(place_index).server_name = action_map.(place_name).server_name;
                obj.place_actions(place_index).package_name = action_map.(place_name).package_name;
                if isempty(find(obj.unique_ROS_package_dependencies == action_map.(place_name).package_name))
                    obj.nROSDependencies = obj.nROSDependencies + 1;
                    obj.unique_ROS_package_dependencies(obj.nROSDependencies) = action_map.(place_name).package_name;
                end
                obj.place_actions(place_index).action_name = action_map.(place_name).action_name;
                obj.place_actions(place_index).message = action_map.(place_name).message;
            end
            %Check all places for initial tag of "r." to differentiate
            %between robot and non-robot places
            nPlaces = size(obj.places, 2);
            for p_index = 1:nPlaces
                place_name = obj.places(p_index);
                is_nonrobot_place = startsWith(place_name, "r.");
                if ~is_nonrobot_place
                    obj.add_robot_place(place_name);
                end
            end
                 
        end
        
        function add_robot_place(obj, place)
            obj.robot_places = cat(2, obj.robot_places, place);
        end
        
        function set_all_places_as_robot_places(obj)
            obj.robot_places = obj.places;
        end
        
        function remove_robot_place(obj, place)
            index = find(obj.robot_places == place);
            obj.robot_places(index) = [];
        end
        
        function add_robots(obj, robot_list, initial_locations)
            if size(robot_list, 2) ~= size(initial_locations, 2)
                error("Each robot must have its initial location defined");
            end
            nInputs = size(robot_list, 2);
            for r_index = 1:nInputs
                init_location = initial_locations(r_index);
                init_place_index = obj.find_place_index(init_location);
                if init_place_index == 0
                    error_string = init_location+ "is not a valid place in the GSPNR";
                    error(error_string);
                end
                obj.robot_initial_locations = cat(2, obj.robot_initial_locations, init_place_index);
            end
            obj.robot_list = cat(2, obj.robot_list, robot_list);
            obj.nRobots = size(obj.robot_list, 2);
        end
        
        function check_robot_ambiguity(obj)
            nTrans = size(obj.transitions, 2);
            for t_index = 1:nTrans
                [input_place_indices, col, val] = find(obj.input_arcs(:,t_index));
                if sum(val)>1
                    disp_string = "There is an input token/robot ambiguity involving transition - "+obj.transitions(t_index);
                    disp(disp_string);
                    obj.ambiguity = true;
                end
                [row, output_place, val] = find(obj.output_arcs(t_index, :));
                if sum(val)>1
                    disp_string = "There is an output token/robot ambiguity involving transition - "+obj.transitions(t_index);
                    disp(disp_string);
                    obj.ambiguity = true;
                end
            end
        end
        
        function check_robot_conservation(obj)
            nTrans = size(obj.transitions, 2);
            for t_index = 1:nTrans
                [input_place_index, col, input_val] = find(obj.input_arcs(:,t_index));
                nInputPlaces = size(input_place_index, 1);
                nInputRobots = 0;
                for pi_index = 1:nInputPlaces
                    input_place_name = obj.places(input_place_index(pi_index));
                    if ~isempty(find(obj.robot_places == input_place_name))
                        %Is a robot place, will remove a robot token
                        nInputRobots = nInputRobots + input_val(pi_index);
                    end
                end
                [row, output_place_index, output_val] = find(obj.output_arcs(t_index, :));
                nOutputPlaces = size(output_place_index, 2);
                nOutputRobots = 0;
                for po_index = 1:nOutputPlaces
                    output_place_name = obj.places(output_place_index(po_index));
                    if ~isempty(find(obj.robot_places == output_place_name))
                        %Is a robot place, will get a robot token
                        nOutputRobots = nOutputRobots + output_val(po_index);
                    end
                end
                if (nInputRobots ~= nOutputRobots)
                    disp_string = "Transition "+obj.transitions(t_index)+" creates or destroys robots";
                    disp(disp_string);
                    obj.robot_conservation = false;
                end
            end
        end
        
        function set_policy(obj, policy_struct)
            obj.policy.markings = policy_struct.markings;
            nMarkings = size(markings, 1);
            for m_index = 1:nMarkings
                state_name = policy_struct.states(m_index);
                state_index = policy_struct.mdp.find_state(state_name);
                action_index = policy_struct.mdp_policy(state_index);
                obj.policy.transitions(m_index) = policy_struct.mdp.actions(action_index);
            end
        end
        
        function transition = get_policy(obj, marking)
            if obj.empty_policy == false
                [exists, marking_index] = ismember(marking, obj.policy.markings, 'rows');
                if exists
                    transition = obj.policy.transitions(marking_index);
                else
                    transition = "";
                end
            else
                transition = "";
            end
        end
        
        function set_empty_policy(obj)
            obj.empty_policy = true;
        end
 
        function simple_exp_trans = find_simple_exp_transitions(obj)
            nTransitions = size(obj.transitions, 2);
            for t_index = 1:nTransitions
                [input_place_indices, col, val] = find(obj.input_arcs(:, t_index));
                nInputPlaces = size(input_place_indices, 1);
                is_simple = true;
                for ip_index = 1:nInputPlaces
                    place_index = input_place_indices(ip_index);
                    place_name = obj.places(place_index);
                    if ~isempty(find(obj.robot_places == place_name))
                        is_simple = false;
                        break;
                    end
                end
                if is_simple == true
                    transition_type = obj.type_transitions(t_index);
                    if transition_type == "exp"
                        obj.simple_exp_transitions = cat(2, obj.simple_exp_transitions, obj.transitions(t_index));
                    end
                end
            end
            simple_exp_trans = obj.simple_exp_transitions;
        end
        
        function delete(obj)
            disp("Deleting temporary files")
            [status, cmdout] = system("echo $CMAKE_PREFIX_PATH");
            path = strtrim(cmdout);
            path = strsplit(path, ":");
            path = path{1};
            path = strrep(path, "devel", "src");
            path = path + "/temp_matlab_gspnr_python_interface";
            bash_cmd = "rm -rf "+path;
            system(bash_cmd);
        end
    end
end

