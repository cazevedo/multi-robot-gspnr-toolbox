clc
clear

clc
clear

%% Creating GSPNR Model

PNPRO_path = "cpr_solar_farm_withglobalcounter.PNPRO";

[nGSPN, models] = ImportfromGreatSPN(PNPRO_path);

NavigationModel = models.navigation;
InspectionModel = models.inspection;

% Creating overall GSPNR model using topological map and imported models

adjacency_matrix = [0  17  0  0   0   0;
                    17 0   11 25  0   0;
                    0  11  0  0   0   0;
                    0  25  0  0   0   29;
                    0  0   0  0   0   16;
                    0  0   0  29  16  0];
 
topological_map = digraph(adjacency_matrix, {'panel1' 'panel2' 'panel3' 'center' 'panel4' 'panel5'}, 'omitselfloops');

%plot(topological_map)

actions_available.panel1 = ["inspection"];
actions_available.panel2 = ["inspection"];
actions_available.panel3 = ["inspection"];
actions_available.panel4 = ["inspection"];
actions_available.panel5 = ["inspection"];
actions_available.center = [string.empty];

robot_marking.center = 3;
GSPNRModel = GSPNRCreationfromTopMap(topological_map, actions_available, models, robot_marking);

global_arc_places = ["r.InspectionsGlobal","r.RequiredInspectionpanel1","r.RequiredInspectionpanel2","r.RequiredInspectionpanel3","r.RequiredInspectionpanel4","r.RequiredInspectionpanel5"];  
global_arc_trans  = ["InspectedAll"       ,"InspectedAll","InspectedAll","InspectedAll","InspectedAll","InspectedAll"];
global_arc_type   = ["in"                 ,"out"         ,"out"        ,"out"       ,"out"      ,"out"          ];
global_arc_weights= [5                    ,1             ,1             ,1             ,1             ,1             ];

GSPNRModel.add_arcs(global_arc_places,global_arc_trans,global_arc_type,global_arc_weights);
%% Loading policy
load("solarfarm_inspection_globalcounter_3robots.mat", "complete_policy");

%% Running simulation
GSPNRModel.evaluate_policy(complete_policy, 1);