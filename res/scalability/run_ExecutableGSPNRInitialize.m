function run_ExecutableGSPNRInitialize(nLocations)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    cd ..;cd ..;cd ..;

    p = genpath('multi-robot-gspnr-toolbox');
    addpath(p);
    
    [GSPNR, parameters] = test_ExecutableGSPNRInitialize(nLocations);
    cd 'multi-robot-gspnr-toolbox/logs/scalability/increasing_locations/';
    filename = "locations"+string(nLocations)+"_execution_initialization_test.mat";
    save(filename, 'GSPNR', 'parameters');
    quit();
end

