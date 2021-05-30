function run_ExecutableGSPNRExecution_increasing_robots(nRobots)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    cd ..;cd ..;cd ..;
    feature('getpid')
    p = genpath('multi-robot-gspnr-toolbox');
    addpath(p);
    cd 'multi-robot-gspnr-toolbox';
    test_ExecutableGSPNRExecution(nRobots);
    quit();
end

