clc
clear

%% Importing GSPNRs from GreatSPN

PNPRO_file = 'execution_tests.PNPRO';

[nGSPN, models] = ImportfromGreatSPN(PNPRO_file);

model = models.small_with_exp;

%% Converting to executable - reading from YAML file

executableModel = ExecutableGSPNR();

YAML_file = 'small_with_exp.yaml';
action_place_struct = ReadfromYAML(YAML_file);

executableModel.initialize(model, action_place_struct);

%% Preparing execution - setting empty policy (random)

executableModel.set_empty_policy();

%% Adding robot, creating interface action servers in temp package

executableModel.add_robots(["robot_0"], ["decision"]);
RobotDistribution = executableModel.robot_initial_locations;
executableModel.create_ros_interface_package();

%% Execute

executableModel.start_execution();