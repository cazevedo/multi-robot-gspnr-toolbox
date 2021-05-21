function run_value_iteration(nLocations)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    cd ..;cd ..;cd ..;
    
    p = genpath('multi-robot-gspnr-toolbox');
    addpath(p);
    cd 'multi-robot-gspnr-toolbox';
    [value_iteration_parameters] = test_value_iteration(nLocations);
    cd 'logs/scalability/increasing_locations/';
    filename = "locations"+string(nLocations)+"_creation_test.mat";
    save(filename, 'value_iteration_parameters', '-append');
    quit();
end

