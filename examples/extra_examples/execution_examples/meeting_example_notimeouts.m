clc
clear

%% Importing GSPN primitive action models (navigation/mopping/vacuuming)

PNPRO_path = "homeclean_notimeouts.PNPRO";

[nGSPN, models] = ImportfromGreatSPN(PNPRO_path);

NavigationModel = models.navigation;
MoppingModel = models.mopping;
VacuumingModel = models.vacuuming;

%% Creating overall GSPNR model using topological map and imported models

adjacency_matrix = [0 1 0;
                    1 0 1;
                    0 1 0];
 
topological_map = digraph(adjacency_matrix, {'L1' 'L2' 'L3'}, 'omitselfloops');

% plot(topological_map)

actions_available.L1 = ["mopping", "vacuuming"];
actions_available.L2 = ["mopping", "vacuuming"];
actions_available.L3 = ["mopping", "vacuuming"];

robot_marking.L1 = 1;
robot_marking.L3 = 1;
GSPNRModel = GSPNRCreationfromTopMap(topological_map, actions_available, models, robot_marking);

% disp("Press any key to continue...");
% pause()

%% Adding transition rewards, and converting GSPNR model to equivalent MDP

% reward_elements = ["Mop_L2"];
% reward_types = ["place"];
% reward_values = [1];
% 
% GSPNRModel.set_reward_functions(reward_elements, reward_values, reward_types);

%% Evaluating optimal policy

% disp("Started computing policy");
% complete_policy = GSPNRModel.policy_synthesis();
% disp("Finished computing policy");
%% Converting GSPNRModel to an executable version - reading from YAML file execution parameters

executableModel = ExecutableGSPNR();
% 
yaml_filepath = 'meeting_example.yaml';
% action_places = ReadfromYAML(yaml_filepath);
executableModel.initialize(GSPNRModel, yaml_filepath, []);

%% Preparing ExecutableGSPNR to execute - remove nonrobot places properties, load policy from .mat file

%Set policy
% policy_workspace_filepath = 'policy_meeting_example_notimeouts_place_rewards.mat';
% load(policy_workspace_filepath, 'complete_policy');

% executableModel.set_policy(complete_policy);
executableModel.set_empty_policy();

%% Preparing executable GSPNR - adding robots, creating interface action servers;

executableModel.add_robots(["robot_0", "robot_1"], ["L1", "L3"]);
RobotDistribution = executableModel.robot_initial_locations;
pwd
executableModel.create_ros_interface_package(true)
pwd
%% Executing the GSPNR
executableModel.start_execution();

