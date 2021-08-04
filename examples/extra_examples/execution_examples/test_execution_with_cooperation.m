clc
clear

[nGSPN, GSPN_list] = ImportfromGreatSPN("test_executable_with_result.PNPRO");

model = GSPN_list.cooperation_test;

% executableModel = ExecutableGSPNR();
% 
% yaml_filepath = 'feedback_test.yaml';
% % action_places = ReadfromYAML(yaml_filepath);
% executableModel.initialize(model, yaml_filepath, []);
% executableModel.add_robots(["jackal0" "warthog"], ["UAV_ready" "UGV_ready"]);
% 
% executableModel.create_ros_interface_package(false);
% 
% executableModel.start_execution();