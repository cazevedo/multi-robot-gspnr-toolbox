clc
clear

[nGSPN, GSPN_list] = ImportfromGreatSPN("test_executable_with_result.PNPRO");

model = GSPN_list.GSPN_exponential;

% executableModel = ExecutableGSPNR();
% 
% yaml_filepath = 'feedback_test.yaml';
% % action_places = ReadfromYAML(yaml_filepath);
% executableModel.initialize(model, yaml_filepath, []);
% executableModel.add_robots(["jackal0"], ["start"]);
% 
% executableModel.create_ros_interface_package(false);
% 
% executableModel.start_execution();