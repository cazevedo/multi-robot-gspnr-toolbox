function run_GSPNRCreationAndConversiontoMDP_increasing_robots(nRobots)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    disp(datestr(now));
    cd ..;cd ..;cd ..;
%     disp("Doing this thing");
%     feature('getpid')
    p = genpath('multi-robot-gspnr-toolbox');
    addpath(p);
    [GSPNR, parameters] = test_GSPNRCreationandConversiontoMDP_increasing_robots(nRobots);
    cd 'multi-robot-gspnr-toolbox/logs/scalability/increasing_robots/';
    filename = "robots"+string(nRobots)+"_creation_test.mat";
    save(filename, 'GSPNR', 'parameters');
%     disp(datestr(now));
    
    quit();
end

