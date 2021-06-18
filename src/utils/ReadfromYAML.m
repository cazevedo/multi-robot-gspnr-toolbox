function [action_place_struct] = ReadfromYAML(yaml_filepath)
%Reads from YAML file and creates struct that holds the ROS message to be sent to action server when place is marked 
%   OUTPUTS
%   map_pl_act_msg      (struct)    (robot_type).(place_name).action_name
%                                   (robot_type).(place_name).message
%                       
    
    YamlStruct = ReadYaml(yaml_filepath);
    
    %Cell Array
    action_places = fieldnames(YamlStruct);
    nActionPlaces = size(action_places,1);
    for a_index = 1:nActionPlaces
        place_name = action_places{a_index};
        server_name     = YamlStruct.(place_name).x0x2Daction_server_name;
        action_name     = YamlStruct.(place_name).x0x2Daction_name;
        package_name    = YamlStruct.(place_name).x0x2Dpackage_name;
        message_fields  = fieldnames(YamlStruct.(place_name).x0x2Dmessage_fields);
        if isfield(YamlStruct.(place_name), 'x0x2Dwith_result')
            with_result = YamlStruct.(place_name).x0x2Dwith_result;
        else
            with_result = false;
        end
        nMsgFields = size(message_fields, 1);
        msg = struct();
        for m_index = 1:nMsgFields
            field_name = message_fields{m_index};
            msg.(field_name) = YamlStruct.(place_name).x0x2Dmessage_fields.(field_name);
        end
        
        action_place_struct.(place_name).server_name = server_name;
        action_place_struct.(place_name).action_name = action_name;
        action_place_struct.(place_name).package_name = package_name;
        action_place_struct.(place_name).message = msg;
        action_place_struct.(place_name).with_result = with_result;
        if with_result
            result_message_fields = fieldnames(YamlStruct.(place_name).x0x2Dresult_trans);
            nRMsgFields = size(result_message_fields,1);
            r_msg = struct();
            for r_index = 1:nRMsgFields
                field_name = result_message_fields{r_index};
                r_msg.(field_name) = YamlStruct.(place_name).x0x2Dresult_trans.(field_name);
            end
            action_place_struct.(place_name).result_trans = r_msg;
        else
            action_place_struct.(place_name).result_trans = [];
        end
            
    end
end

