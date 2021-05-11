clear
clc

%% Parsing for increasing locations
max_nLocations = 42;

%Prepare arrays to hold data (index will be number of locations for that
%test)
nPlaces = zeros(1, max_nLocations);
nTransitions = zeros(1, max_nLocations);
nEdges = zeros(1, max_nLocations);
GSPNR_creation_time = zeros(1, max_nLocations);
MDP_creation_time = zeros(1, max_nLocations);
nStates = zeros(1, max_nLocations);
exec_init_time = zeros(1, max_nLocations);

%Parse all MAT files;
for nLocations = 1:max_nLocations
    mat_filename = "locations"+string(nLocations)+"_creation_test.mat";
    load(mat_filename, 'parameters', 'pre_execution_parameters');
    nPlaces(nLocations) = parameters.nPlaces;
    nTransitions(nLocations) = parameters.nTransitions;
    nEdges(nLocations) = parameters.nEdges;
    GSPNR_creation_time(nLocations) = parameters.GSPNR_creation_time;
    MDP_creation_time(nLocations) = parameters.MDP_creation_time;
    nStates(nLocations) = parameters.nStates;
    exec_init_time(nLocations) = pre_execution_parameters.initialization_time;
end
%Parse CPU/mem log file;
logID = fopen("creation_log_cpu_mem.txt", 'r');
formatSpec = '%d,%f,%f';
sizeA = [3 Inf];
creation_cpu_mem = fscanf(logID,formatSpec, sizeA);

logID = fopen("preexecution_log_cpu_mem.txt", 'r');
formatSpec = '%d,%f,%f';
sizeA = [3 Inf];
preexecution_cpu_mem = fscanf(logID,formatSpec, sizeA);

logID = fopen("execution_log_cpu_mem.txt", 'r');
formatSpec = '%d,%f,%f';
sizeA = [3 Inf];
execution_cpu_mem = fscanf(logID,formatSpec, sizeA);

%% Plotting
close all
plot(MDP_creation_time, 'b')
scatter(linspace(1,max_nLocations,max_nLocations), MDP_creation_time, 'red', 'x')
plot(MDP_creation_time, 'b')
hold on
grid minor
grid
scatter(linspace(1,max_nLocations,max_nLocations), MDP_creation_time, 'red', 'x')
xlabel('Number of locations')
ylabel('Time spent converting GSPN into equivalent MDP [seconds]')
title('Conversion from GSPNR Model to equivalent MDP')

figure;
plot(GSPNR_creation_time, 'b')
scatter(linspace(1,max_nLocations,max_nLocations), GSPNR_creation_time, 'red', 'x')
xlabel('Number of locations')
plot(GSPNR_creation_time, 'b')
grid minor
grid on
hold on
scatter(linspace(1,max_nLocations,max_nLocations), GSPNR_creation_time, 'red', 'x')
xlabel('Number of locations')
ylabel('Time spent creating GSPN model [seconds]')
title('GSPNR Model Creation')

figure;
plot(nStates, 'b')
hold on
grid minor
grid on
scatter(linspace(1,max_nLocations,max_nLocations), nStates, 'red', 'x')
xlabel('Number of locations')
ylabel('Number of states in equivalent MDP')
title('Number of states in equivalent MDP')

figure;
plot(creation_cpu_mem(1,:), creation_cpu_mem(2,:), 'b');
grid minor
grid on
hold on
scatter(creation_cpu_mem(1,:), creation_cpu_mem(2,:), 'red', 'x')
xlabel('Number of locations')
ylabel('Average CPU usage [% of a single core]')
title('GSPNR Model Creation - Average CPU usage')

figure;
plot(creation_cpu_mem(1,:), creation_cpu_mem(3,:), 'b');
grid minor
grid on
hold on
scatter(creation_cpu_mem(1,:), creation_cpu_mem(3,:), 'red', 'x')
xlabel('Number of locations')
ylabel('Average memory usage []')
title('GSPNR Model Creation - Average memory usage')

figure;
plot(nPlaces, 'b');
grid minor
grid on
hold on
scatter(linspace(1,max_nLocations,max_nLocations) , nPlaces, 'red', 'x')
xlabel('Number of locations')
ylabel('Number of places in the GSPNR Model')
title('GSPNR Model Creation - number of places')

figure;
plot(execution_cpu_mem(1,:), execution_cpu_mem(2,:), 'b');
grid minor
grid on
hold on
scatter(execution_cpu_mem(1,:), execution_cpu_mem(2,:), 'red', 'x')
xlabel('Number of locations')
ylabel('Average CPU usage [% of a single core]')
title('GSPNR Execution - Average CPU usage')

%% Parsing for increasing robots

max_nRobots = 4;

%Prepare arrays to hold data (index will be number of locations for that
%test)
robot_nPlaces = zeros(1, max_nRobots);
robot_nTransitions = zeros(1, max_nRobots);
robot_nEdges = zeros(1, max_nRobots);
robot_GSPNR_creation_time = zeros(1, max_nRobots);
robot_MDP_creation_time = zeros(1, max_nRobots);
robot_nStates = zeros(1, max_nRobots);
exec_init_time = zeros(1, max_nRobots);

%Parse all MAT files;
for nRobots = 1:max_nRobots
    mat_filename = "robots"+string(nRobots)+"_creation_test.mat";
    load(mat_filename, 'parameters')%, 'pre_execution_parameters');
    robot_nPlaces(nRobots) = parameters.nPlaces;
    robot_nTransitions(nRobots) = parameters.nTransitions;
    robot_nEdges(nRobots) = parameters.nEdges;
    robot_GSPNR_creation_time(nRobots) = parameters.GSPNR_creation_time;
    robot_MDP_creation_time(nRobots) = parameters.MDP_creation_time;
    robot_nStates(nRobots) = parameters.nStates;
    %exec_init_time(nRobots) = pre_execution_parameters.initialization_time;
end
%Parse CPU/mem log file;
logID = fopen("creation_log_cpu_mem.txt", 'r');
formatSpec = '%d,%f,%f';
sizeA = [3 Inf];
robot_creation_cpu_mem = fscanf(logID,formatSpec, sizeA);