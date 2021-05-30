function [executableModel, parameters] = test_ExecutableGSPNRInitialize_increasing_robots(nRobots)
    
    parameters = struct();
    %Importing GSPNR Model from previous run test
    mat_filename = "robots"+string(nLocations)+"_creation_test.mat";
    load(mat_filename, 'GSPNR', 'action_place');
    executableModel = ExecutableGSPNR();
    
    tic
    executableModel.initialize(GSPNR, [], action_place);
    parameters.initialization_time = toc;
    
    executableModel.set_empty_policy();
    robot_names = [string.empty];
    robot_locations = [string.empty];
    for r_index = 1:nRobots
        int = r_index - 1;
        name = "robot_"+string(int);
        robot_names = cat(2, robot_names, name);
        if mod(r_index, 2)~=0
            robot_locations = cat(2, robot_locations, "L1");
        else
            robot_locations = cat(2, robot_locations, "L4");
        end
    end
    executableModel.add_robots(robot_names, robot_locations);
    RobotDistribution = executableModel.robot_initial_locations;
    executableModel.set_catkin_ws("/home/harode/tiago_ws");
    executableModel.create_ros_interface_package(false)

end

