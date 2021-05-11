function run_ExecutableGSPNRExecution(nLocations)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    cd ..;cd ..;cd ..;
    feature('getpid')
    p = genpath('multi-robot-gspnr-toolbox');
    addpath(p);
    cd 'multi-robot-gspnr-toolbox';
    test_ExecutableGSPNRExecution(nLocations);
    quit();
end

