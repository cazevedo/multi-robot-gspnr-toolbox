function launch_name = create_python_interface_scripts(obj, package_dir)
    nScripts = obj.nRobots;
    script_paths = package_dir + "/src/";
    disp("Scripts path when creating - ");
    disp(script_paths)
    for s_index = 1:nScripts
        robot_name = obj.robot_list(s_index);
        script_name = "matlab_interface_server_"+robot_name;
        script_location = "~/"+ script_paths + script_name
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
                server_name = robot_name + "/" + obj.place_actions(p_index).server_name;
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
                fprintf(fileID, '\n\t%i : [%s, %s(), %s()], ', p_index, action_name, goal_msg_name, result_msg_name);
            end
        end
        fprintf(fileID, '\n\t}');
        fprintf(fileID, '\n\treturn switcher.get(index)');
        
        fprintf(fileID, '\n\ndef place_index_to_results(index):');
        fprintf(fileID, '\n\tswitcher = {');
        for p_index = 1:nPlaces
            with_result = obj.place_actions(p_index).with_result;
            result = obj.place_actions(p_index).result_trans;
            if (~isempty(with_result) && with_result==1)
                result_msg_fields = string(fieldnames(result));
                nFields = size(result_msg_fields,1);
                dict = "{";
                for f_index = 1:nFields
                    field_name = result_msg_fields(f_index);
                    field_content = result.(field_name);
                    trans_index = obj.find_transition_index(field_content);
                    dict = dict+"'"+field_name+"'"+":"+string(trans_index)+", ";
                end
                dict = dict+"}, ";
                fprintf(fileID, '\n\t%i : %s', p_index, dict);
            end
        end
        fprintf(fileID, '\n\t}');
        fprintf(fileID, '\n\tif index in switcher.keys():\n\t\treturn switcher.get(index)');
        fprintf(fileID, '\n\telse:\n\t\treturn False');
        
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
        fprintf(fileID, '\n\t\tslave_result = place_index_to_action_msgs(place_index)[2]');
        fprintf(fileID, '\n\t\tgoal_dict = place_index_to_goal(place_index)');
        fprintf(fileID, '\n\t\tfor key, value in goal_dict.items():');
        fprintf(fileID, '\n\t\t\t setattr(slave_goal, key, value)');
        fprintf(fileID, '\n\t\tclient.send_goal(slave_goal)\n\t\tclient.wait_for_result()');
        fprintf(fileID, '\n\t\tif place_index_to_results(place_index)!=False:');
        fprintf(fileID, '\n\t\t\tslave_result = client.get_result()');
        fprintf(fileID, '\n\t\t\ttrans_index = place_index_to_results(place_index)[slave_result.result]');
        fprintf(fileID, '\n\t\t\tresult.sequence.append(place_index)\n\t\t\tresult.sequence.append(robot_index)\n\t\t\tresult.sequence.append(trans_index)');
        fprintf(fileID, '\n\t\telse:');
        fprintf(fileID, '\n\t\t\tclient.get_result()\n\t\t\tresult.sequence.append(place_index)\n\t\t\tresult.sequence.append(robot_index)\n\t\t\tresult.sequence.append(0)');
        fprintf(fileID, '\n\t\tself._as.set_succeeded(result)');
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
    launch_path = package_dir + "/launch/";
    launch_name = "matlab_interface_servers.launch";
    launch_file_location = "~/"+ launch_path + launch_name;
    fileID = fopen(launch_file_location, 'w');
    fprintf(fileID, "<?xml version=""1.0""?>\n<launch>");
    for r_index = 1:obj.nRobots
        content = "\n\t <node pkg = ""temp_matlab_gspnr_python_interface"" type= """ +erase(obj.interface_action_servers(r_index),"/")+""" name="""+erase(obj.interface_action_servers(r_index),"/")+"""/>";
        fprintf(fileID, content);
    end
    fprintf(fileID, "\n</launch>");
    fclose(fileID);
end
