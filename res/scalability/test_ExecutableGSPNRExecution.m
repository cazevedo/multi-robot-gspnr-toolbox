function test_ExecutableGSPNRExecution(nLocations)
    cleanupObj = onCleanup(@CleaningAfterInterrupt);
    feature('getpid')
    %Importing GSPNR Model from previous run test
    mat_filename = "locations"+string(nLocations)+"_creation_test.mat";
    load(mat_filename, 'executable');
    executable.set_marking(executable.initial_marking)
    executable.create_ros_interface_package(true);
    pause(20);
    disp("STARTING EXECUTION");
    executable.start_execution();
end

function CleaningAfterInterrupt()
    disp("Exiting MATLAB");
    quit();
end

