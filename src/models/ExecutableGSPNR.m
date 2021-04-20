classdef ExecutableGSPNR < GSPNR
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = private)
        unique_ROS_package_dependencies = [string.empty];
        nROSDependencies = 0;
        place_actions = struct();
        policy = struct();
        ambiguity = false;
        robot_conservation = true;
        robot_list = [string.empty];
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
                 
        end
        
        function add_robot_place(obj, place)
            obj.robot_places = cat(2, obj.robot_places, place)
        end
        
        function set_all_places_as_robot_places(obj)
            obj.robot_places = obj.places;
        end
        
        function remove_robot_place(obj, place)
            index = find(obj.robot_places == place);
            obj.robot_places(index) = [];
        end
        
        function add_robots(obj, robot_list)
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
        
        function set_policy(obj, markings, states, mdp, mdp_policy)
            obj.policy.markings = markings;
            nMarkings = size(markings, 1);
            for m_index = 1:nMarkings
                state_name = states(m_index);
                state_index = mdp.find_state(state_name);
                action_index = mdp_policy(state_index);
                obj.policy.transitions(m_index) = mdp.actions(action_index);
            end
        end
        
        function transition = get_policy(obj, marking)
            [exists, marking_index] = ismember(marking, obj.policy.markings, 'rows');
            if exists
                transition = obj.policy.transitions(marking_index);
            else
                transition = "";
            end
        end
        
        function create_python_interface_scripts(obj, package_path)
            nScripts = obj.nRobots;
            script_paths = package_path + "/src/";
            for s_index = 1:nScripts
                robot_name = obj.robot_list(s_index);
                script_name = "matlab_interface_server_"+robot_name+".py";
                script_location = script_paths + script_name;
                fileID = fopen(script_location, 'w');
                fprintf(fileID, '#! /usr/bin/env python\nimport rospy\nimport actionlib\nimport actionlib_tutorials.msg\n');
                for d_index = 1:obj.nROSDependencies
                    package_name = obj.unique_ROS_package_dependencies(d_index);
                    package_msg = package_name + ".msg";
                    fprintf(fileID, 'import %s\n', package_msg);
                end
                fprintf(fileID, '\nrobot_name = ''%s''', robot_name);
                fprintf(fileID, '\nrobot_index = %i', s_index);
                fprintf(fileID, '\n\ndef place_index_to_action_server(index):');
                fprintf(fileID, '\n\tswitcher = {');
                nPlaces = size(obj.place_actions,2);
                for p_index = 1:nPlaces
                    if ~isempty(obj.place_actions(p_index).place_name)
                        server_name = robot_name + "_" + obj.place_actions(p_index).server_name;
                        fprintf(fileID, '\n\t%i : ''%s'',', p_index, server_name);
                    end
                end
                fprintf(fileID, '\n\t}');
                fprintf(fileID, '\n\treturn switcher.get(index)');
                fprintf(fileID, '\n\ndef place_index_to_goal(index):');
                fprintf(fileID, '\n\tswitcher = {');
                for p_index = 1:nPlaces
                    msg = obj.place_actions(p_index).message;
                    if ~isempty(msg)
                        msg_fields = string(fieldnames(msg));
                        nFields = size(msg_fields,1);
                        msg_dict = "{";
                        for f_index = 1:nFields
                            field_name = msg_fields(f_index);
                            field_content = msg.(field_name);
                            msg_dict = msg_dict+"'"+field_name+"'"+":"+string(field_content)+", ";
                        end
                        msg_dict = msg_dict+"}, ";
                        fprintf(fileID, '\n\t%i : %s', p_index, msg_dict);
                    end
                end
                fprintf(fileID, '\n\t}');
                fprintf(fileID, '\n\treturn switcher.get(index)');
                fprintf(fileID, '\n\ndef place_index_to_action_msgs(index):');
                fprintf(fileID, '\n\tswitcher = {');
                for p_index = 1:nPlaces
                    if ~isempty(obj.place_actions(p_index).package_name)
                        package_name = obj.place_actions(p_index).package_name;
                        package_messages = package_name+".msg.";
                        action_name = obj.place_actions(p_index).action_name;
                        action_name = package_messages+action_name;
                        goal_msg_name = strrep(action_name, "Action", "Goal");
                        result_msg_name = strrep(action_name, "Action", "Result");
                        fprintf(fileID, '\n\t%i : [%s, %s(), %s], ', p_index, action_name, goal_msg_name, result_msg_name);
                    end
                end
                fprintf(fileID, '\n\t}');
                fprintf(fileID, '\n\treturn switcher.get(index)');
                fprintf(fileID, '\n\nclass MatlabInterfaceAction(object):');
                fprintf(fileID, '\n\n\tdef __init__(self, name):\n\t\tself._action_name = name\n\t\tself._as = actionlib.SimpleActionServer(self._action_name, actionlib_tutorials.msg.FibonacciAction, execute_cb=self.execute_cb, auto_start = False)\n\t\tself._as.start()');
                fprintf(fileID, '\n\n\tdef execute_cb(self, goal):\n\t\tplace_index = goal.order');
                fprintf(fileID, '\n\t\trospy.loginfo(''Received a goal'') ');
                fprintf(fileID, '\n\t\tresult = actionlib_tutorials.msg.FibonacciResult()');
                fprintf(fileID, '\n\t\taction_server = place_index_to_action_server(place_index)');
                fprintf(fileID, '\n\t\taction_type = place_index_to_action_msgs(place_index)[0]');
                fprintf(fileID, '\n\t\tclient = actionlib.SimpleActionClient(action_server, action_type)');
                fprintf(fileID, '\n\t\tclient.wait_for_server()');
                fprintf(fileID, '\n\t\tslave_goal = place_index_to_action_msgs(place_index)[1]');
                fprintf(fileID, '\n\t\tgoal_dict = place_index_to_goal(place_index)');
                fprintf(fileID, '\n\t\tfor key, value in goal_dict.iteritems():');
                fprintf(fileID, '\n\t\t\t setattr(slave_goal, key, value)');
                fprintf(fileID, '\n\t\tclient.send_goal(slave_goal)\n\t\tclient.wait_for_result()\n\t\tclient.get_result()\n\t\tresult.sequence.append(place_index)\n\t\tresult.sequence.append(robot_index)\n\t\tself._as.set_succeeded(result)');
                fprintf(fileID, '\n\nif __name__ == ''__main__'':');
                
                interface_server_node_name = "rospy.init_node('matlab_interface_server_"+robot_name+"')";
                fprintf(fileID, '\n\t%s', interface_server_node_name);
                fprintf(fileID, '\n\tserver = MatlabInterfaceAction(rospy.get_name())');
                fprintf(fileID, '\n\trospy.spin()');
                fclose(fileID);
                make_executable = "chmod +x "+script_location;
                status = system(make_executable);
                obj.interface_action_servers(s_index) = erase(script_name, ".py");
                obj.interface_action_servers(s_index) = "/"+obj.interface_action_servers(s_index);
            end
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
                    obj.simple_exp_transitions = cat(2, obj.simple_exp_transitions, obj.transitions(t_index));
                end
            end
            simple_exp_trans = obj.simple_exp_transitions;
        end
            
    end
end

