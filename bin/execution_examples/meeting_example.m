clc
clear

%% Importing GSPN primitive action models (navigation/mopping/vacuuming)

PNPRO_path = "meeting_example.PNPRO";

[nGSPN, models] = ImportfromGreatSPN(PNPRO_path);

NavigationModel = models.navigation;
MoppingModel = models.mopping;
VacuumingModel = models.vacuuming;

%% Creating overall GSPNR model using topological map and imported models

adjacency_matrix = [0 1 0;
                    1 0 1;
                    0 1 0];
 
topological_map = digraph(adjacency_matrix, {'L1' 'L2' 'L3'}, 'omitselfloops');

plot(topological_map)

actions_available.L1 = ["mopping", "vacuuming"];
actions_available.L2 = ["mopping", "vacuuming"];
actions_available.L3 = ["mopping", "vacuuming"];

robot_marking.L1 = 1;
robot_marking.L3 = 1;
GSPNRModel = GSPNRCreationfromTopMap(topological_map, actions_available, models, robot_marking);

disp("Press any key to continue...");
pause()

%% Adding transition rewards, and converting GSPNR model to equivalent MDP
% 
% reward_elements = ["Mop_L1", "Mop_L2", "Mop_L3", "Vacuum_L1", "Vacuum_L2", "Vacuum_L3"];
% reward_types = repmat("transition", [1 6]);
% reward_values = ones(1, 6);
% 
% GSPNRModel.set_reward_functions(reward_elements, reward_values, reward_types);
% 
% [mdp, markings, states, types] = GSPNRModel.toMDP();
% mdp.check_validity();
% mdp.set_enabled_actions();
% 
% %% Evaluating optimal policy
% 
% [values, policy] = value_iteration(mdp, 1, 0.99, 0.01);

%% Converting GSPNRModel to an executable version - reading from YAML file execution parameters

executableModel = ExecutableGSPNR();

yaml_filepath = 'meeting_example.yaml';

action_place_struct = ReadfromYAML(yaml_filepath);

executableModel.import_nonExecutable(GSPNRModel, action_place_struct);

%% Preparing ExecutableGSPNR to execute - remove nonrobot places from robot places list, load policy from .mat file

executableModel.set_all_places_as_robot_places();

%Remove nonrobot places from robot places
executableModel.remove_robot_place("NotRequiredMopL1");
executableModel.remove_robot_place("NotRequiredMopL2");
executableModel.remove_robot_place("NotRequiredMopL3");
executableModel.remove_robot_place("RequiredMopL3");
executableModel.remove_robot_place("RequiredMopL2");
executableModel.remove_robot_place("RequiredMopL1");
executableModel.remove_robot_place("RequiresVacuumL1");
executableModel.remove_robot_place("RequiresVacuumL2");
executableModel.remove_robot_place("RequiresVacuumL3");
executableModel.remove_robot_place("NotRequiredVacuumL3");
executableModel.remove_robot_place("NotRequiredVacuumL2");
executableModel.remove_robot_place("NotRequiredVacuumL1");

executableModel.check_robot_ambiguity()
executableModel.check_robot_conservation()

%Set policy
% policy_workspace_filepath = 'meeting_example_policy_workspace.mat';
% load(policy_workspace_filepath, 'markings', 'states', 'mdp', 'policy');
%
%executableModel.set_policy(markings, states, mdp, policy);
executableModel.set_empty_policy();


%% Preparing executable GSPNR - adding robots, creating interface action servers;

executableModel.add_robots(["robot_0", "robot_1"]);
L1_index = executableModel.find_place_index("L1");
L3_index = executableModel.find_place_index("L3");
RobotDistribution = [L1_index, L3_index];
executableModel.create_ros_interface_package()
disp("Please start up the Python Interface Scripts. After starting them up, press any key to continue...");
pause()

%% Executing the GSPNR
ROSExecutionManager(executableModel, RobotDistribution);

