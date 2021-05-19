function [executableModel, parameters] = test_ExecutableGSPNRInitialize(nLocations)
    
    parameters = struct();
    %Importing GSPNR Model from previous run test
    mat_filename = "locations"+string(nLocations)+"_creation_test.mat";
    load(mat_filename, 'GSPNR', 'action_place');
    executableModel = ExecutableGSPNR();
    
    tic
    executableModel.initialize(GSPNR, [], action_place);
    parameters.initialization_time = toc;
    
    executableModel.set_empty_policy();
    
    name_last_location = "L"+string(nLocations);
    executableModel.add_robots(["robot_0", "robot_1"], ["L1", name_last_location]);
    RobotDistribution = executableModel.robot_initial_locations;
    executableModel.set_catkin_ws("/home/harode/tiago_ws");
    executableModel.create_ros_interface_package(true)

end

