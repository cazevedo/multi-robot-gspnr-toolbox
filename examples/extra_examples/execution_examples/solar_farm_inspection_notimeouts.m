clc
clear

%% Importing GSPN primitive action models (navigation/mopping/vacuuming)

PNPRO_path = "cpr_solar_farm_notimeouts.PNPRO";

[nGSPN, models] = ImportfromGreatSPN(PNPRO_path);

NavigationModel = models.navigation;
InspectionModel = models.inspection;

%% Creating overall GSPNR model using topological map and imported models

adjacency_matrix = [0  17  0  0   0   0;
                    17 0   11 25  0   0;
                    0  11  0  0   0   0;
                    0  25  0  0   0   29;
                    0  0   0  0   0   16;
                    0  0   0  29  16  0];
 
topological_map = digraph(adjacency_matrix, {'panel1' 'panel2' 'panel3' 'center' 'panel4' 'panel5'}, 'omitselfloops');

plot(topological_map)

actions_available.panel1 = ["inspection"];
actions_available.panel2 = ["inspection"];
actions_available.panel3 = ["inspection"];
actions_available.panel4 = ["inspection"];
actions_available.panel5 = ["inspection"];
actions_available.center = [string.empty];

robot_marking.center = 3;
GSPNRModel = GSPNRCreationfromTopMap(topological_map, actions_available, models, robot_marking);


%% Adding transition rewards, and converting GSPNR model to equivalent MDP

reward_elements = ["Inspect_panel1", "Inspect_panel2", "Inspect_panel3", "Inspect_panel4", "Inspect_panel5" ];
reward_types = ["transition", "transition", "transition", "transition", "transition"];
reward_values = [1, 1, 1, 1, 1];

GSPNRModel.set_reward_functions(reward_elements, reward_values, reward_types);

%% Evaluating optimal policy

disp("Started computing policy");
complete_policy = GSPNRModel.policy_synthesis();
disp("Finished computing policy");

% %% Converting GSPNRModel to an executable version - reading from YAML file execution parameters
% 
% executableModel = ExecutableGSPNR();
% % 
% yaml_filepath = 'meeting_example.yaml';
% % action_places = ReadfromYAML(yaml_filepath);
% executableModel.initialize(GSPNRModel, yaml_filepath, []);
% 
% %% Preparing ExecutableGSPNR to execute - remove nonrobot places properties, load policy from .mat file
% 
% %Set policy
% % policy_workspace_filepath = 'policy_meeting_example_notimeouts_place_rewards.mat';
% % load(policy_workspace_filepath, 'complete_policy');
% 
% % executableModel.set_policy(complete_policy);
% executableModel.set_empty_policy();
% 
% %% Preparing executable GSPNR - adding robots, creating interface action servers;
% 
% executableModel.add_robots(["robot_0", "robot_1"], ["L1", "L3"]);
% RobotDistribution = executableModel.robot_initial_locations;
% pwd
% executableModel.create_ros_interface_package(true)
% pwd
% %% Executing the GSPNR
% executableModel.start_execution();

