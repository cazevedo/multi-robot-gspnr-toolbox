clc
clear

[nGSPN, GSPN_list] = ImportfromGreatSPN("test_executable_with_result.PNPRO");

model = GSPN_list.cooperation_test;

executableModel = ExecutableGSPNR();

yaml_filepath = 'cooperation_test.yaml';
% action_places = ReadfromYAML(yaml_filepath);
executableModel.initialize(model, yaml_filepath, []);
executableModel.add_robots(["jackal0" "warthog"], ["UAV_ready" "UGV_ready"]);

places = [string.empty];
types = [string.empty];
nPlaces = size(executableModel.places, 2);
for p_index = 1:nPlaces
    place_name = executableModel.places(p_index);
    if contains(place_name, "UAV")
        places = cat(2, places, place_name);
        types = cat(2, types, "UAV");
    elseif contains(place_name, "UGV")
        places = cat(2, places, place_name);
        types = cat(2, types, "UGV");
    end
end

executableModel.set_types(["UAV", "UGV"], places, types); 

executableModel.set_catkin_ws("catkin_ws");
executableModel.create_ros_interface_package(false);

executableModel.start_execution();