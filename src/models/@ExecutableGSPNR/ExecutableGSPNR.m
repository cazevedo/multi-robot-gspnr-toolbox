classdef ExecutableGSPNR < GSPNR
    %UNTITLED Model for executable version of GSPNR, mapping tokens to robots
    %   Detailed explanation goes here
    
    properties (SetAccess = private)
        catkin_ws = string.empty;      %[string] - absolute path to catkin workspace where interface ROS package will be created and built
        unique_ROS_package_dependencies = [string.empty]; %[string array] - list of names of unique ROS packages used by user-defined ROS action servers
        nROSDependencies = 0;           %[int] - number of unique ROS packages used by user-defined ROS action servers
        launch_cmd = string.empty;      %[string] - command sent to OS to launch interface action servers
        
        place_actions = struct();       %[struct array] - each index points to the action server and further information needed to send goal corresponding to the execution of the particular place with that index

        policy = struct();              %[struct]
        empty_policy = true;            %[logical] - true if policy has not been set, and false otherwise

        ambiguity = false;              %[logical] - true if GSPNR model has ambiguity regarding mapping tokens to robots (cannot execute if true)
        robot_conservation = true;      %[logical] - true if GSPNR model does not create robots or destroy robots
        
        robot_list = [string.empty];    %[string array] - each string is the ROS namespace of each robot (ordered)
        robot_initial_locations = [];   %[int array] - each element indicates the place index where each robot starts out
        nRobots = 0;                    %[int] - number of robots
        
        robot_places = [string.empty];  %[int array] - each element indicates the place index where each token/robot currently is
        
        interface_action_servers = [string.empty]; %[string array] - each element indicates the name of the interface action server corresponding to each robot
        
        simple_exp_transitions = [];            %[string array] - list of exponential transitions not involved with robot places, that can fire independently
        simple_exp_transition_flags = [];       %[string array] - elements are either "EXE" or "FIN", indicating if there is a timer already running for each simple exponential transition in the simple_exp_transitions property

    end
    
    methods
        function execGSPNR = ExecutableGSPNR()
            execGSPNR = execGSPNR@GSPNR();
        end
        function set_place_actions(obj, action_map)
            obj.place_actions = action_map;
        end
        function initialize(obj, GSPNR, YAML_filepath, action_map)
            %Initialize instance of object with an already existing GSPNR model;
            %Input:
            %   GSPNR           [GSPNR object] - non executable version to be imported;
            %   YAML_filepath   [string]       - string containing path of YAML file to use when initializing instance;
            %   action_map      [struct]       - struct containing where each fieldname is the name of a place, and the value is the
            %                                    struct containing the name of the ROS package, name of action server, and goal
            %                                    message of the action place;
            
            %If YAML_filepath input is not empty, function will disregard
            %completely the third input "action_map";
            %Copy properties of non-executable instance;
            copy_gspn = copy(GSPNR);
            obj.places = copy_gspn.places;
            nPlaces = size(obj.places, 2);
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
            %Initialize place_actions propertyd
            basic_element = struct('place_name', [], 'server_name', [], 'package_name', [], 'action_name', [], 'message', [], 'with_result', [], 'result_trans', []);
            obj.place_actions = basic_element;
            for p_index = 1:(nPlaces-1)
                obj.place_actions = [obj.place_actions, basic_element];
            end
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
                obj.place_actions(place_index).with_result = action_map.(place_name).with_result;
                if action_map.(place_name).with_result == 1
                    obj.place_actions(place_index).result_trans = action_map.(place_name).result_trans;
                end
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
            %Set already existing place as a robot place
            %Input:
            %   place [string] - name of place to be set as robot place
            obj.robot_places = cat(2, obj.robot_places, place);
        end
        
        function set_all_places_as_robot_places(obj)
            %Set all existing places as robot places
            obj.robot_places = obj.places;
        end
        
        function remove_robot_place(obj, place)
            %Remove already existing place from robot places property
            %Input:
            %   place [string] - name of place to remove as robot place
            index = find(obj.robot_places == place);
            obj.robot_places(index) = [];
        end
        
        function add_robots(obj, robot_list, initial_locations)
            %Add robots to executable model
            %Input:
            %   robot_list          [string array] - namespaces of robots to add
            %   initial_locations   [int array] - initial place indexes of robots that are being added
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
            %Checks ambiguity regarding the token/robot mapping
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
            %Checks that model does not create or destroy tokens that are mapped to robots
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
        
        function set_manual_policy(obj, markings, transitions)
            %Sets handcrafted policy
            %Input:
            %   markings    [int matrix] - each row is a marking for the GSPNR
            %   transitions [string array] - each i-th element is the transition that should fire when the GSPNR's current marking is the
            %                                marking in the i-th row of the markings input
            obj.policy.markings = markings;
            obj.transitions = transitions;
        end
        
        function set_policy(obj, policy_struct)
            %Sets policy computed by the policy synthesis module
            %Input:
            %   policy_struct   [struct] - struct containing the following fields:
            %                       * state_index_to_markings - matrix where each row is a GSPNR marking, and the index corresponds
            %                       to the index of the corresponding state of the equivalent MDP model;
            %                       * states - string array, where the elements are ordered according to their corresponding index
            %                       * mdp_policy - string array, where each element corresponds to the transition that should fire
            %                       when the MDP model is in the state of the element's index;
            %                       * mdp - MDP object where policy was computed on;
            obj.empty_policy = false;
            obj.policy.markings = policy_struct.state_index_to_markings;
            nMarkings = size(obj.policy.markings, 1);
            for m_index = 1:nMarkings
                state_name = policy_struct.states(m_index);
                state_index = policy_struct.mdp.find_state(state_name);
                action_index = policy_struct.mdp_policy(state_index);
                obj.policy.transitions(m_index) = policy_struct.mdp.actions(action_index);
            end
        end
        
        function transition = get_policy(obj, marking)
            %Returns the transition chosen by the policy for a given marking
            %Input:
            %   marking     [int array] - marking for the GSPNR
            %Output:
            %   transition  [string] - transition to fire;
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
            %Sets policy as empty
            obj.empty_policy = true;
        end
 
        function simple_exp_trans = find_simple_exp_transitions(obj)
            %Finds all exponential transitions that do not involve robot places and can fire independently
            %Output:
            %   simple_exp_trans    [string array] - list of all simple exponential transitions;
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
        
        function set_catkin_ws(obj, catkin_ws)
            %Set catkin workspace path, if empty create_ros_interface_package() method and create_ros_interface_scripts() method will fallback to CMAKE_PREFIX_PATH environment variable
            %Input:
            %   catkin_ws   [string] - absolute path to catkin workspace where ROS interface package will be created and built
            obj.catkin_ws = catkin_ws;
        end
        
        function delete(obj)
            disp("Deleting temporary files")
            if isempty(obj.catkin_ws)
                [status, cmdout] = system("echo $CMAKE_PREFIX_PATH");
                path = strtrim(cmdout);
                path = strsplit(path, ":");
                path = path{1};
                path = strrep(path, "devel", "src");
                path = path + "/temp_matlab_gspnr_python_interface";
                bash_cmd = "rm -rf "+path;
                system(bash_cmd);
            else
                path = obj.catkin_ws + "/src/temp_matlab_gspnr_python_interface";
                bash_cmd = "rm -rf "+path;
                system(bash_cmd);
            end
            for r_index = 1:obj.nRobots
                kill_cmd = "rosnode kill /matlab_interface_server_"+obj.robot_list(r_index);
                system(kill_cmd);
            end
        end
    end
end

