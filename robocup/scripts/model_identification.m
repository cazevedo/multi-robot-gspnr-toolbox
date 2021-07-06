clc
clear

load("data_ModelIdentification.mat");

rates = struct();

%% Retrieving charging times

low_level_charging_starting_times = results.timestamps(find(results.transitions == "Charge_BS_B0"));
low_level_charging_finish_times = results.timestamps(find(results.transitions == "Charged_BS_B0"));

low_level_charging_times = seconds(low_level_charging_finish_times - low_level_charging_starting_times);

rates.charge.B0 = mean(low_level_charging_times);

medium_level_charging_starting_times = results.timestamps(find(results.transitions == "Charge_BS_B1"));
medium_level_charging_finish_times = results.timestamps(find(results.transitions == "Charged_BS_B1"));

medium_level_charging_times = seconds(medium_level_charging_finish_times - medium_level_charging_starting_times);

rates.charge.B1 = mean(medium_level_charging_times);

%% Retrieving navigation times and discharging probabilities

%Use topological map to iterate through edges and battery levels

discharging_probabilities = struct();

battery_levels = 3;
battery_level_names = ["B0", "B1", "B2"];

adjacency_matrix = [0 1 0 0 0 0 0;
                    1 0 1 1 1 1 0;
                    0 1 0 0 0 0 0;
                    0 1 0 0 0 0 0;
                    0 1 0 0 0 0 0;
                    0 1 0 0 0 0 1;
                    0 0 0 0 0 1 0];
 
topological_map = digraph(adjacency_matrix, {'BS' 'L1' 'L2' 'L3' 'L4' 'L5' 'L6'}, 'omitselfloops');

edges = string(topological_map.Edges.EndNodes);
weights = topological_map.Edges.Weight;
nEdges = size(edges, 1);
for e_index = 1:nEdges
    source = edges(e_index, 1);
    target = edges(e_index, 2);
    rate = 1/weights(e_index);
    for b_level = 1:battery_levels
        battery_level = battery_level_names(b_level);
        if battery_level == "B0"
            go_transition = "Go_"+source+"_"+target+"_"+battery_level;
            arrived_transition = "Arrived_"+source+"_"+target+"_"+battery_level;
            starting_times = results.timestamps(find(results.transitions == go_transition));
            end_times = results.timestamps(find(results.transitions == arrived_transition));
            nEndTimes = size(end_times, 1);
            starting_times = starting_times(1:nEndTimes);
            navigation_times = seconds(end_times - starting_times);
            mean_time = mean(navigation_times);
            rates.navigation.(source+target).(battery_level) = mean_time;
        end
        if battery_level == "B1"
            go_transition = "Go_"+source+"_"+target+"_"+battery_level;
            arrived_transition = "Arrived_"+source+"_"+target+"_"+battery_level;
            starting_times = results.timestamps(find(results.transitions == go_transition));
            end_times = results.timestamps(find(results.transitions == arrived_transition));
            nEndTimes = size(end_times, 1);
            starting_times = starting_times(1:nEndTimes);
            navigation_times = seconds(end_times - starting_times);
            mean_time = mean(navigation_times);
            rates.navigation.(source+target).(battery_level) = mean_time;
            
            same_level_transition = "Bat_Level_"+source+"_"+target+"_B1_B1";
            below_level_transition = "Bat_Level_"+source+"_"+target+"_B1_B0";
            nStaySameLevel = size(find(results.transitions == same_level_transition), 1);
            nBelowLevel = size(find(results.transitions == below_level_transition), 1);
            total = nStaySameLevel + nBelowLevel;
            discharging_probabilities.navigation.(source+target).(battery_level).stay_same_level = nStaySameLevel/total;
            discharging_probabilities.navigation.(source+target).(battery_level).below_level = nBelowLevel/total;
            discharging_probabilities.navigation.(source+target).(battery_level).total = total;
        end
        if battery_level == "B2"
            go_transition = "Go_"+source+"_"+target+"_"+battery_level;
            arrived_transition = "Arrived_"+source+"_"+target+"_"+battery_level;
            starting_times = results.timestamps(find(results.transitions == go_transition));
            end_times = results.timestamps(find(results.transitions == arrived_transition));
            nEndTimes = size(end_times, 1);
            starting_times = starting_times(1:nEndTimes);
            navigation_times = seconds(end_times - starting_times);
            mean_time = mean(navigation_times);
            rates.navigation.(source+target).(battery_level) = mean_time;
            
            same_level_transition = "Bat_Level_"+source+"_"+target+"_B2_B2";
            below_level_transition = "Bat_Level_"+source+"_"+target+"_B2_B1";
            nStaySameLevel = size(find(results.transitions == same_level_transition), 1);
            nBelowLevel = size(find(results.transitions == below_level_transition), 1);
            total = nStaySameLevel + nBelowLevel;
            discharging_probabilities.navigation.(source+target).(battery_level).stay_same_level = nStaySameLevel/total;
            discharging_probabilities.navigation.(source+target).(battery_level).below_level = nBelowLevel/total;
            discharging_probabilities.navigation.(source+target).(battery_level).total = total;
        end
    end
end

%% Retrieving vacuuming times and discharging probabilities

nodes = string(table2array(topological_map.Nodes));
nNodes = size(nodes, 1);

%Creating vacuuming models in each location, except in BS (base station)
%and L1
for n_index = 1:nNodes
    node_name = nodes(n_index);
    if node_name == "BS" || node_name == "L1"
        continue
    end
    for b_level = 1:battery_levels
        battery_level = battery_level_names(b_level);
        if battery_level == "B0"
            %No vacuuming when robots is in lowest battery level
            continue
        end
        if battery_level == "B1"
            start_transition = "Vacuum_"+node_name+"_"+battery_level;
            end_transition = "Finished_Vacuum_"+node_name+"_"+battery_level;
            starting_times = results.timestamps(find(results.transitions == start_transition));
            end_times = results.timestamps(find(results.transitions == end_transition));
            nEndTimes = size(end_times, 1);
            starting_times = starting_times(1:nEndTimes);
            vacuum_times = seconds(end_times - starting_times);
            mean_time = mean(vacuum_times);
            rates.vacuum.(node_name).(battery_level) = mean_time;
            
            same_level_transition = "Bat_Level_"+source+"_B1_B1";
            below_level_transition = "Bat_Level_"+source+"_B1_B0";
            nStaySameLevel = size(find(results.transitions == same_level_transition), 1);
            nBelowLevel = size(find(results.transitions == below_level_transition), 1);
            total = nStaySameLevel + nBelowLevel
            discharging_probabilities.vacuum.(node_name).(battery_level).stay_same_level = nStaySameLevel/total;
            discharging_probabilities.vacuum.(node_name).(battery_level).below_level = nBelowLevel/total;
            discharging_probabilities.vacuum.(node_name).(battery_level).total = total;
            
        end
        if battery_level == "B2"
            start_transition = "Vacuum_"+node_name+"_"+battery_level;
            end_transition = "Finished_Vacuum_"+node_name+"_"+battery_level;
            starting_times = results.timestamps(find(results.transitions == start_transition));
            end_times = results.timestamps(find(results.transitions == end_transition));
            nEndTimes = size(end_times, 1);
            starting_times = starting_times(1:nEndTimes);
            vacuum_times = seconds(end_times - starting_times);
            mean_time = mean(vacuum_times);
            rates.vacuum.(node_name).(battery_level) = mean_time;
            
            same_level_transition = "Bat_Level_"+source+"_B2_B2";
            below_level_transition = "Bat_Level_"+source+"_B2_B1";
            nStaySameLevel = size(find(results.transitions == same_level_transition), 1);
            nBelowLevel = size(find(results.transitions == below_level_transition), 1);
            total = nStaySameLevel + nBelowLevel;
            discharging_probabilities.vacuum.(node_name).(battery_level).stay_same_level = nStaySameLevel/total;
            discharging_probabilities.vacuum.(node_name).(battery_level).below_level = nBelowLevel/total;
            discharging_probabilities.vacuum.(node_name).(battery_level).total = total;
        end
    end
end

%% Averaging out mean times for same action at different battery levels

navigation_actions = string(fieldnames(rates.navigation));
nNavigationActions = size(navigation_actions, 1);

for index = 1:nNavigationActions
    action_name = navigation_actions(index);
    B0_time = rates.navigation.(action_name).B0;
    B1_time = rates.navigation.(action_name).B1;
    B2_time = rates.navigation.(action_name).B2;
    rates.navigation.(action_name) = mean([B0_time B1_time B2_time], 'omitnan');
end

vacuum_actions = string(fieldnames(rates.vacuum));
nNavigationActions = size(vacuum_actions, 1);

for index = 1:nNavigationActions
    action_name = vacuum_actions(index);
    B1_time = rates.vacuum.(action_name).B1;
    B2_time = rates.vacuum.(action_name).B2;
    rates.vacuum.(action_name) = mean([B1_time B2_time], 'omitnan');
end

save("model_parameters.mat", "rates", "discharging_probabilities");



