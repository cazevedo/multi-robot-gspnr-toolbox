clear
clc

max_nLocations = 42;

%Prepare arrays to hold data (index will be number of locations for that
%test)
nPlaces = zeros(1, max_nLocations);
nTransitions = zeros(1, max_nLocations);
nEdges = zeros(1, max_nLocations);
GSPNR_creation_time = zeros(1, max_nLocations);
MDP_creation_time = zeros(1, max_nLocations);
nStates = zeros(1, max_nLocations);
CPU_usage = zeros(1, max_nLocations);
mem_usage = zeros(1, max_nLocations);
%Parse all MAT files;
for nLocations = 1:max_nLocations
    mat_filename = "locations"+string(nLocations)+"_creation_test.mat";
    load(mat_filename, 'parameters');
    nPlaces(nLocations) = parameters.nPlaces;
    nTransitions(nLocations) = parameters.nTransitions;
    nEdges(nLocations) = parameters.nEdges;
    GSPNR_creation_time(nLocations) = parameters.GSPNR_creation_time;
    MDP_creation_time(nLocations) = parameters.MDP_creation_time;
    nStates(nLocations) = parameters.nStates;
end
%Parse CPU/mem log file;
logID = fopen("creation_log_cpu_mem.txt", 'r');
formatSpec = '%d,%f,%f';
sizeA = [3 Inf];
cpu_mem = fscanf(logID,formatSpec, sizeA);

