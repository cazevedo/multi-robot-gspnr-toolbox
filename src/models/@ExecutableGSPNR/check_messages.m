function check_messages(obj)
%CHECK_MESSAGES Summary of this function goes here
%   Detailed explanation goes here
    nActionPlaces = size(obj.place_actions, 2);
    all_messages = rosmsg('list');
    for p_index = 1:nActionPlaces
        if isempty(obj.place_actions(p_index).place_name)
            continue;
        end
        package_name = obj.place_actions(p_index).package_name;
        action_name = obj.place_actions(p_index).action_name;
        full_action_name = package_name+'/'+action_name;
        if ~any(strcmp(all_messages, full_action_name))
            error_msg = "Need to import action "+action_name+" from package "+action_name;
            error(error_msg);
        else
            continue;
        end
    end
    disp("All messages needed are correctly imported!");
    obj.messages_check = true;
end

