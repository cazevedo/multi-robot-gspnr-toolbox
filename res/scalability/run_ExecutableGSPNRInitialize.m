function run_ExecutableGSPNRInitialize(nLocations)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    cd ..;cd ..;cd ..;
    pwd
    p = genpath('multi-robot-gspnr-toolbox');
    addpath(p);
    cd 'multi-robot-gspnr-toolbox';
    [executable, pre_execution_parameters] = test_ExecutableGSPNRInitialize(nLocations);
    cd 'logs/scalability/increasing_locations/';
    filename = "locations"+string(nLocations)+"_creation_test.mat";
    save(filename, 'executable', 'pre_execution_parameters', '-append');
    pause();
    quit();
end

