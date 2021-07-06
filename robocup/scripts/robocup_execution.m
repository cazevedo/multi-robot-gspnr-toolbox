clc
clear

%% Loading files
load("robocup_corrected_models.mat");
load("robocup_corrected_policy", "policy");

%% Setting up policy
policy_struct.gspn = homeModel;
policy_struct.state_index_to_markings = markings;
policy_struct.states = states;
policy_struct.mdp = mdp;
policy_struct.mdp_policy = policy;

%% Changing immediate transitions to exponential transitions - for 'Bat_Levels...'
homeModel_imm = copy(homeModel);

check_bat_trans_indices = find(startsWith(homeModel_imm.transitions, "Bat_Level"));

for ti_index = 1:size(check_bat_trans_indices, 2)
    trans_index = check_bat_trans_indices(ti_index);
    trans_name = homeModel_imm.transitions(trans_index);
    homeModel_imm.change_type_of_transition(trans_name, "exp");
end

%% Setting up map between action places and ROS action servers

action_map = struct();

[exp_trans, exp_trans_indices] = homeModel_imm.get_exponential_transitions();

nEXPTrans = size(exp_trans, 2);

action_places = [];

for t_index = 1:nEXPTrans
    exp_trans_index = exp_trans_indices(t_index);
    input_places = homeModel_imm.input_arcs(:,exp_trans_index);
    input_place_indices = find(input_places);
    action_places = cat(2, action_places, input_place_indices');
end
action_places = unique(action_places);

%Package name where ROS action servers are
package_name = "multi_robot_home_clean";

nActionPlaces = size(action_places, 2);

for p_index = 1:nActionPlaces
    place_index = action_places(p_index);
    place_name = homeModel_imm.places(place_index);
    if startsWith(place_name, "Charging")
        action_map.(place_name).server_name = "ChargeActionServer";
        action_map.(place_name).package_name = package_name;
        action_map.(place_name).action_name = "ChargeAction";
        action_map.(place_name).message = struct("charge", 1);
        action_map.(place_name).with_result = false;
    end
    if startsWith(place_name, "Navigating")
        action_map.(place_name).server_name = "NavigateWaypointActionServer";
        action_map.(place_name).package_name = package_name;
        action_map.(place_name).action_name = "NavigateWaypointAction";
        components = strsplit(place_name, "_");
        origin_string = components(2);
        destination_string = components(3);
        action_map.(place_name).message = struct("origin", origin_string, "destination", destination_string);
        action_map.(place_name).with_result = false;
    end
    if startsWith(place_name, "Check_Bat")
        action_map.(place_name).server_name = "CheckBatteryActionServer";
        action_map.(place_name).package_name = package_name;
        action_map.(place_name).action_name = "CheckBatteryAction";
        action_map.(place_name).message = struct("check", 1);
        
        components = strsplit(place_name, "_");
        if size(components,2) == 4
            action_map.(place_name).with_result = true;
            %Discharge after vacuum
            current_battery_level = components(4);
            if current_battery_level == "B2"
                lower_battery_level = "B1";
                transition_to_same_level = "Bat_Level_"+components(3)+"_"+components(4)+"_"+components(4);
                transition_to_level_below = "Bat_Level_"+components(3)+"_"+components(4)+"_"+lower_battery_level;
                action_map.(place_name).result_trans = struct("high", transition_to_same_level, "medium", transition_to_level_below);
            elseif current_battery_level == "B1"
                lower_battery_level = "B0";
                transition_to_same_level = "Bat_Level_"+components(3)+"_"+components(4)+"_"+components(4);
                transition_to_level_below = "Bat_Level_"+components(3)+"_"+components(4)+"_"+lower_battery_level;
                action_map.(place_name).result_trans = struct("medium", transition_to_same_level, "low", transition_to_level_below);
            end
        elseif size(components,2) == 5
            %Discharge after navigation
            current_battery_level = components(5);
            if current_battery_level == "B2"
                action_map.(place_name).with_result = true;
                lower_battery_level = "B1";
                transition_to_same_level = "Bat_Level_"+components(3)+"_"+components(4)+"_"+components(5)+"_"+components(5);
                transition_to_level_below = "Bat_Level_"+components(3)+"_"+components(4)+"_"+components(5)+"_"+lower_battery_level;
                action_map.(place_name).result_trans = struct("high", transition_to_same_level, "medium", transition_to_level_below);
            elseif current_battery_level == "B1"
                action_map.(place_name).with_result = true;
                lower_battery_level = "B0";
                transition_to_same_level = "Bat_Level_"+components(3)+"_"+components(4)+"_"+components(5)+"_"+components(5);
                transition_to_level_below = "Bat_Level_"+components(3)+"_"+components(4)+"_"+components(5)+"_"+lower_battery_level;
                action_map.(place_name).result_trans = struct("medium", transition_to_same_level, "low", transition_to_level_below);
            elseif current_battery_level == "B0"
                action_map.(place_name).with_result = false;
            end
        end
        
    end
    if startsWith(place_name, "Vacuuming")
        action_map.(place_name).server_name = "InspectWaypointActionServer";
        action_map.(place_name).package_name = package_name;
        action_map.(place_name).action_name = "InspectWaypointAction";
        components = strsplit(place_name, "_");
        waypoint_string = components(2);
        action_map.(place_name).message = struct("waypoint", waypoint_string);
        action_map.(place_name).with_result = false;
    end
end


%% Creating executable

executable_homeModel = ExecutableGSPNR();

executable_homeModel.initialize(homeModel_imm, [], action_map);

initial_place_index = executable_homeModel.find_place_index("BS_B2")

%executable_homeModel.set_policy(policy_struct);
