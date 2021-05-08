function run_GSPNRCreationAndConversiontoMDP(nLocations)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    disp(datestr(now));
    cd ..;cd ..;cd ..;
%     disp("Doing this thing");
%     feature('getpid')
    p = genpath('multi-robot-gspnr-toolbox');
    addpath(p);
    [GSPNR, parameters] = test_GSPNRCreationandConversiontoMDP(nLocations);
    cd 'multi-robot-gspnr-toolbox/logs/scalability/increasing_locations/';
    filename = "locations"+string(nLocations)+"_creation_test.mat";
    save(filename, 'GSPNR', 'parameters');
%     disp(datestr(now));
    quit();
end

