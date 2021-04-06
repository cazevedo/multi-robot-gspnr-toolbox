function [action_place_struct] = PrepareROSExecution(yaml_filepath)
%Reads from YAML file and creates struct that holds the ROS message to be sent to action server when place is marked 
%   OUTPUTS
%   map_pl_act_msg      (struct)    (robot_type).(place_name).action_name
%                                   (robot_type).(place_name).message
%                       
    
    YamlStruct = ReadYaml(yaml_filepath);
    
    %Cell Array
    robot_types = fieldnames(YamlStruct);
    n_types = size(robot_types, 1);
    
    for t_index = 1:n_types
        robot_type = robot_types{t_index};
        action_places = fieldnames(YamlStruct.(robot_type));
        nActionPlaces = size(action_places,1);
        for a_index = 1:nActionPlaces
            place_name = action_places{a_index};
            action_name     = YamlStruct.(robot_type).(place_name).x0x2Daction_name;
            message_type    = YamlStruct.(robot_type).(place_name).x0x2Dmessage_type;
            msg = rosmessage(message_type);
            message_fields  = fieldnames(YamlStruct.(robot_type).(place_name).x0x2Dmessage_fields);
            nMsgFields = size(message_fields, 1);
            for m_index = 1:nMsgFields
                field_name = message_fields{m_index};
                msg.(field_name) = YamlStruct.(robot_type).(place_name).x0x2Dmessage_fields.(field_name);
                action_place_struct.(robot_type).(place_name).action_name = action_name;
                action_place_struct.(robot_type).(place_name).message = msg;
            end
        end
    end
end

