clc
clear

%% Topological Map for Solar Farm Scenario
adjacency_matrix = [0  17  0  0   0   0;
                    17 0   11 25  0   0;
                    0  11  0  0   0   0;
                    0  25  0  0   0   29;
                    0  0   0  0   0   16;
                    0  0   0  29  16  0];
 
topological_map = digraph(adjacency_matrix, {'panel1' 'panel2' 'panel3' 'center' 'panel4' 'panel5'}, 'omitselfloops');
plot(topological_map)

%% Importing Models from GreatSPN

PNPRO_path = 'video_GSPNR.PNPRO';
[nGSPN, GSPN_list] = ImportfromGreatSPN(PNPRO_path);

LocationsModel  = GSPN_list.heterogeneous_team;

NavigationModelFullBattery = GSPN_list.navigation_fullbattery;
NavigationModelHalfBattery = GSPN_list.navigation_halffullbattery;

UGVNavigationModel         = GSPN_list.navigation_UGV;

InspectionModelFullBattery = GSPN_list.inspection_fullbattery;
InspectionModelHalfBattery = GSPN_list.inspection_halffullbattery;

ChargingModel   = GSPN_list.charging;

%% Building GSPNR object

solarfarm = GSPNR();

battery_levels = 3;
battery_level_names = ["B0", "B1", "B2"];
%Inspection
nodes = string(table2array(topological_map.Nodes));
nNodes = size(nodes, 1);
for n_index = 1:nNodes
    node_name = nodes(n_index);
    if node_name == "center"
        continue
    else
        for b_level = 1:battery_levels
            battery_level = battery_level_names(b_level);
            if battery_level == "B0"
                continue
            end
            if battery_level == "B1"
                inspection_halfbattery = copy(InspectionModelHalfBattery);
                inspection_halfbattery.format([node_name, "B1", "B0"]);
                solarfarm = MergeGSPNR(solarfarm, inspection_halfbattery);
            end
            if battery_level == "B2"
                inspectionfullbattery = copy(InspectionModelFullBattery);
                inspectionfullbattery.format([node_name, "B2", "B1"]);
                solarfarm = MergeGSPNR(solarfarm, inspectionfullbattery);
            end
        end
    end
            
end
%Charging
for n_index = 1:nNodes
    node_name = nodes(n_index);
    for b_level = 1:battery_levels
        battery_level = battery_level_names(b_level);
        if battery_level == "B0"
            charging = copy(ChargingModel);
            charging.format([node_name, battery_level, "B2"]);
            solarfarm = MergeGSPNR(solarfarm, charging);
        end
        if battery_level == "B1"
            charging = copy(ChargingModel);
            charging.format([node_name, battery_level, "B2"]);
            solarfarm = MergeGSPNR(solarfarm, charging);
        end
        if battery_level == "B2"
            continue;
        end
    end
end
% Navigation
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
            continue;
        end
        if battery_level == "B1"
            navigation_halfbattery = copy(NavigationModelHalfBattery);
            navigation_halfbattery.change_rate_of_transition("Arrived_<1>_<2>_<3>", rate);
            navigation_halfbattery.format([source, target, "B1", "B0"]);            
            solarfarm = MergeGSPNR(solarfarm, navigation_halfbattery);
        end
        if battery_level == "B2"
            navigation_fullbattery = copy(NavigationModelFullBattery);
            navigation_fullbattery.change_rate_of_transition("Arrived_<1>_<2>_<3>", rate);
            navigation_fullbattery.format([source, target, "B2", "B1"]);
            solarfarm = MergeGSPNR(solarfarm, navigation_fullbattery);
        end
    end
    ugv_navigation = copy(UGVNavigationModel);
    ugv_navigation.change_rate_of_transition("Arrived_<1>_<2>_UGV", rate);
    ugv_navigation.format([source, target]);
    solarfarm = MergeGSPNR(solarfarm, ugv_navigation);
end

global_arc_places = ["r.InspectionsGlobal","r.RequiredInspectionpanel1","r.RequiredInspectionpanel2","r.RequiredInspectionpanel3","r.RequiredInspectionpanel4","r.RequiredInspectionpanel5"];  
global_arc_trans  = ["InspectedAll"       ,"InspectedAll","InspectedAll","InspectedAll","InspectedAll","InspectedAll"];
global_arc_type   = ["in"                 ,"out"         ,"out"        ,"out"       ,"out"      ,"out"          ];
global_arc_weights= [5                    ,1             ,1             ,1             ,1             ,1             ];

solarfarm.add_arcs(global_arc_places,global_arc_trans,global_arc_type,global_arc_weights);

reward_elements = ["Inspect_panel1_B2","Inspect_panel1_B1",...
                   "Inspect_panel2_B2","Inspect_panel2_B1",...
                   "Inspect_panel3_B2","Inspect_panel3_B1",...
                   "Inspect_panel4_B2","Inspect_panel4_B1",...
                   "Inspect_panel5_B2","Inspect_panel5_B1",...
                   "panel1_B0",...
                   "panel2_B0",...
                   "panel3_B0",...
                   "panel4_B0",...
                   "panel5_B0"];
reward_types = ["transition", "transition",...
                "transition", "transition",...
                "transition", "transition",...
                "transition", "transition",...
                "transition", "transition",...
                "place",...
                "place",...
                "place",...
                "place",...
                "place"];
reward_values = [1, 1,...
                 1, 1,...
                 1, 1,...
                 1, 1,...
                 1, 1,...
                -1,...
                -1,...
                -1,...
                -1,...
                -1];

solarfarm.set_reward_functions(reward_elements, reward_values, reward_types);

%Set robots initial locations
initial_marking = solarfarm.initial_marking;
center_index = solarfarm.find_place_index("center_B2");
panel1_index_UGV = solarfarm.find_place_index("panel1_UGV");

%Place Jackals
initial_marking(center_index) = 2;
%Place Warthog
initial_marking(panel1_index_UGV) = 1;


%Set initial counter, all panels need inspection
needs_inspection = solarfarm.places(find(startsWith(solarfarm.places, "r.RequiredInspection")));
for place_name_index = 1:size(needs_inspection, 2)
    place_name = needs_inspection(place_name_index);
    place_index = solarfarm.find_place_index(place_name);
    initial_marking(place_index) = 1;
end

solarfarm.set_initial_marking(initial_marking);

[mdp, markings, states, types] = solarfarm.toMDP_without_wait();