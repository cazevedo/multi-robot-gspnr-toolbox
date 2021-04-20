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

GSPNRModel = GSPNRCreationfromTopMap(topological_map, actions_available, models);

%Place a robot in location L1 and L3

L1_index = GSPNRModel.find_place_index("L1");
L3_index = GSPNRModel.find_place_index("L3");

new_marking = GSPNRModel.current_marking;

new_marking(L1_index) = 1;
new_marking(L3_index) = 1;

GSPNRModel.set_initial_marking(new_marking);
% 
% %% Adding transition rewards, and converting GSPNR model to equivalent MDP
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

% executableModel = ExecutableGSPNR();
% 
% yaml_filepath = "meeting_example.yaml";
% 
% action_place_struct = ReadfromYAML(yaml_filepath);
% 
% executableModel.import_nonExecutable(GSPNRModel, action_place_struct);
% %Set policy
% 
% %% Preparing executable GSPNR - adding robots, creating interface action servers;
% catkin_package_path = "catkin_ws/src/actionlib_experiments";
% 
% executableModel.add_robots(["turtlebot1", "turtlebot2"]);
% RobotDistribution = [L1_index, L3_index];
% executableModel.create_python_interface_scripts(catkin_package_path)
% 
% %% Executing the GSPNR
% ROSExecutionManager(executableModel, RobotDistribution);

