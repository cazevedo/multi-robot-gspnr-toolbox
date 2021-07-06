clc
clear

load("model_parameters.mat", "rates", "discharging_probabilities");

%% Topological Map
adjacency_matrix = [0 8 0 0 0 0 0;
                    8 0 15 18 35 51 0;
                    0 15 0 0 0 0 0;
                    0 18 0 0 0 0 0;
                    0 35 0 0 0 0 0
                    0 51 0 0 0 0 19.5
                    0 0 0 0 0 19.5 0];
 
topological_map = digraph(adjacency_matrix, {'BS' 'L1' 'L2' 'L3' 'L4' 'L5' 'L6'}, 'omitselfloops');
%plot(topological_map)

%% Importing GreatSPN Models
PNPRO_path = 'robocup.PNPRO';
[nGSPN, GSPN_list] = ImportfromGreatSPN(PNPRO_path);

ChargeLowModel = GSPN_list.home_charge_low;
ChargeHalfModel = GSPN_list.home_charge_half;

NavigationFullModel = GSPN_list.home_navigation_full;
NavigationHalfModel = GSPN_list.home_navigation_half;
NavigationLowModel = GSPN_list.home_navigation_low;

VacuumFullModel = GSPN_list.home_vacuum_full;
VacuumHalfModel = GSPN_list.home_vacuum_half;

%% Creating GSPNR Model

homeModel = GSPNR();

battery_levels = 3;
battery_level_names = ["B0", "B1", "B2"];

%Getting list of nodes in topological map
nodes = string(table2array(topological_map.Nodes));
nNodes = size(nodes, 1);

%Creating vacuuming models in each location, except in BS (base station)
%and L1
for n_index = 1:nNodes
    node_name = nodes(n_index);
    if (node_name == "BS" || node_name == "L1")
        %No vacuuming in these locations
        continue
    else
        rate = 1/rates.vacuum.(node_name);
        for b_level = 1:battery_levels
            battery_level = battery_level_names(b_level);
            if battery_level == "B0"
                %No vacuuming when robots is in lowest battery level
                continue
            end
            if battery_level == "B1"
                model = copy(VacuumHalfModel);
                model.change_rate_of_transition("Finished_Vacuum_<1>_<3>", rate);
                weight_same_level = discharging_probabilities.vacuum.(node_name).(battery_level).stay_same_level;
                if weight_same_level == 0
                    weight_same_level = 0.001;
                end
                weight_below_level = discharging_probabilities.vacuum.(node_name).(battery_level).below_level;
                if weight_below_level == 0
                    weight_below_level = 0.001;
                end
                model.change_rate_of_transition("Bat_Level_<1>_<3>_<3>", weight_same_level);
                model.change_rate_of_transition("Bat_Level_<1>_<3>_<4>", weight_below_level);
                model.format([node_name, missing, "B1", "B0"]);
                homeModel = MergeGSPNR(homeModel, model);
            end
            if battery_level == "B2"
                model = copy(VacuumFullModel);
                model.change_rate_of_transition("Finished_Vacuum_<1>_<3>", rate);
                weight_same_level = discharging_probabilities.vacuum.(node_name).(battery_level).stay_same_level;
                if weight_same_level == 0
                    weight_same_level = 0.001;
                end
                weight_below_level = discharging_probabilities.vacuum.(node_name).(battery_level).below_level;
                if weight_below_level == 0
                    weight_below_level = 0.001;
                end
                model.change_rate_of_transition("Bat_Level_<1>_<3>_<3>", weight_same_level);
                model.change_rate_of_transition("Bat_Level_<1>_<3>_<4>", weight_below_level);
                model.format([node_name, missing, "B2", "B1"]);
                homeModel = MergeGSPNR(homeModel, model);
            end
        end
    end
end
%Creating charging model in location "BS" - base station - enable charging
%from medium battery and low
for b_level = 1:battery_levels
    battery_level = battery_level_names(b_level);
    if battery_level == "B2"
        continue
    end
    if battery_level == "B1"
        model = copy(ChargeHalfModel);
        rate = 1/rates.charge.B1;
        model.change_rate_of_transition("Charged_<1>_<3>", rate);
        model.format(["BS", missing, "B1", "B2"]);
        homeModel = MergeGSPNR(homeModel, model);
    end
    if battery_level == "B0"
        model = copy(ChargeLowModel);
        rate = 1/rates.charge.B0;
        model.change_rate_of_transition("Charged_<1>_<3>", rate);
        model.format(["BS", missing, "B0", "B2"]);
        homeModel = MergeGSPNR(homeModel,model);
    end
end
%Creating navigation models for each edge of the topological map and for
%each battery level
edges = string(topological_map.Edges.EndNodes);
weights = topological_map.Edges.Weight;
nEdges = size(edges, 1);
for e_index = 1:nEdges
    source = edges(e_index, 1);
    target = edges(e_index, 2);
    rate = 1/rates.navigation.(source+target);
    for b_level = 1:battery_levels
        battery_level = battery_level_names(b_level);
        if battery_level == "B0"
            model = copy(NavigationLowModel);
            model.change_rate_of_transition("Arrived_<1>_<2>_<3>", rate);
            model.format([source, target, "B0", missing]);
            homeModel = MergeGSPNR(homeModel, model);
        end
        if battery_level == "B1"
            model = copy(NavigationHalfModel);
            model.change_rate_of_transition("Arrived_<1>_<2>_<3>", rate);
            weight_same_level = discharging_probabilities.navigation.(source+target).(battery_level).stay_same_level;
            if weight_same_level == 0
                weight_same_level = 0.001;
            end
            weight_below_level = discharging_probabilities.navigation.(source+target).(battery_level).below_level;
            if weight_below_level == 0
                weight_below_level = 0.001;
            end
            model.change_rate_of_transition("Bat_Level_<1>_<2>_<3>_<3>", weight_same_level);
            model.change_rate_of_transition("Bat_Level_<1>_<2>_<3>_<4>", weight_below_level);
            model.format([source, target, "B1", "B0"]);
            homeModel = MergeGSPNR(homeModel, model);
        end
        if battery_level == "B2"
            model = copy(NavigationFullModel);
            model.change_rate_of_transition("Arrived_<1>_<2>_<3>", rate);
            weight_same_level = discharging_probabilities.navigation.(source+target).(battery_level).stay_same_level;
            if weight_same_level == 0
                weight_same_level = 0.001;
            end
            weight_below_level = discharging_probabilities.navigation.(source+target).(battery_level).below_level;
            if weight_below_level == 0
                weight_below_level = 0.001;
            end
            model.change_rate_of_transition("Bat_Level_<1>_<2>_<3>_<3>", weight_same_level);
            model.change_rate_of_transition("Bat_Level_<1>_<2>_<3>_<4>", weight_below_level);
            model.format([source, target, "B2", "B1"]);
            homeModel = MergeGSPNR(homeModel, model);
        end
    end
end

global_arc_places = ["r.Number_Vacuumed","r.Requires_Vacuum_L2","r.Requires_Vacuum_L3","r.Requires_Vacuum_L4","r.Requires_Vacuum_L5","r.Requires_Vacuum_L6"];  
global_arc_trans  = ["VacuumedAll"        ,"VacuumedAll","VacuumedAll","VacuumedAll","VacuumedAll","VacuumedAll"];
global_arc_type   = ["in"                 ,"out"         ,"out"        ,"out"       ,"out"      ,"out"          ];
global_arc_weights= [5                    ,1             ,1             ,1             ,1             ,1             ];

homeModel.add_arcs(global_arc_places,global_arc_trans,global_arc_type,global_arc_weights);

reward_elements = ["Vacuum_L1_B2","Vacuum_L1_B1",...
                   "Vacuum_L2_B2","Vacuum_L2_B1",...
                   "Vacuum_L3_B2","Vacuum_L3_B1",...
                   "Vacuum_L4_B2","Vacuum_L4_B1",...
                   "Vacuum_L5_B2","Vacuum_L5_B1",...
                   "L1_B0",...
                   "Navigating_L1_BS_B0","Bat_Level_L1_BS_B0",...
                   "Navigating_BS_L1_B0","Bat_Level_BS_L1_B0",...
                   "Navigating_L1_L2_B0","Bat_Level_L1_L2_B0",...
                   "Navigating_L2_L1_B0","Bat_Level_L2_L1_B0",...
                   "Navigating_L1_L3_B0","Bat_Level_L1_L3_B0",...
                   "Navigating_L3_L1_B0","Bat_Level_L3_L1_B0",...
                   "Navigating_L1_L4_B0","Bat_Level_L1_L4_B0",...
                   "Navigating_L4_L1_B0","Bat_Level_L4_L1_B0",...
                   "Navigating_L1_L5_B0","Bat_Level_L1_L5_B0",...
                   "Navigating_L5_L1_B0","Bat_Level_L5_L1_B0",...
                   "L2_B0",...
                   "L3_B0",...
                   "L4_B0",...
                   "L5_B0",...
                   "Navigating_L5_L6_B0","Bat_Level_L5_L6_B0",...
                   "Navigating_L6_L5_B0","Bat_Level_L6_L5_B0"];
reward_types = ["transition", "transition",...
                "transition", "transition",...
                "transition", "transition",...
                "transition", "transition",...
                "transition", "transition",...
                "place",...
                "place","place",...
                "place","place",...
                "place","place",...
                "place","place",...
                "place","place",...
                "place","place",...
                "place","place",...
                "place","place",...
                "place","place",...
                "place","place",...
                "place",...
                "place",...
                "place",...
                "place",...
                "place","place",...
                "place","place"];
reward_values = [1, 1,...
                 1, 1,...
                 1, 1,...
                 1, 1,...
                 1, 1,...
                -1,   ...
                -1, -1,...
                -1, -1,...
                -1, -1,...
                -1, -1,...
                -1, -1,...
                -1, -1,...
                -1, -1,...
                -1, -1,...
                -1, -1,...
                -1, -1,...
                -1,...
                -1,...
                -1,...
                -1,...
                -1, -1,...
                -1, -1];
            
 homeModel.set_reward_functions(reward_elements, reward_values, reward_types);
 
 initial_marking = homeModel.initial_marking;
 needs_vacuum = homeModel.places(find(startsWith(homeModel.places, "r.Requires_Vacuum")));
for place_name_index = 1:size(needs_vacuum, 2)
    place_name = needs_vacuum(place_name_index);
    place_index = homeModel.find_place_index(place_name);
    initial_marking(place_index) = 1;
end

base_full_charge_index = homeModel.find_place_index("BS_B2");

%Place robots
initial_marking(base_full_charge_index) = 2;

homeModel.set_initial_marking(initial_marking);

tic
[mdp, markings, states, types] = homeModel.toMDP_without_wait();
toc    
          