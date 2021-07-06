clear
clc

harode_total_memory = 16.576/100; %GB

%% Parsing for increasing locations
% max_nLocations = 42;
% 
% %Prepare arrays to hold data (index will be number of locations for that
% %test)
% nPlaces = zeros(1, max_nLocations);
% nTransitions = zeros(1, max_nLocations);
% nEdges = zeros(1, max_nLocations);
% GSPNR_creation_time = zeros(1, max_nLocations);
% MDP_creation_time = zeros(1, max_nLocations);
% nStates = zeros(1, max_nLocations);
% %exec_init_time = zeros(1, max_nLocations);
% 
% %Parse all MAT files;
% for nLocations = 1:max_nLocations
%     %nLocations
%     mat_filename = "locations"+string(nLocations)+"_creation_test.mat";
%     load(mat_filename, 'parameters', 'pre_execution_parameters');
%     nPlaces(nLocations) = parameters.nPlaces;
%     nTransitions(nLocations) = parameters.nTransitions;
%     nEdges(nLocations) = parameters.nEdges;
%     GSPNR_creation_time(nLocations) = parameters.GSPNR_creation_time;
%     MDP_creation_time(nLocations) = parameters.MDP_creation_time;
%     nStates(nLocations) = parameters.nStates;
%     %exec_init_time(nLocations) = pre_execution_parameters.initialization_time;
% end
% %Parse value iteration parameters
% for nLocations = 1:39
%     %nLocations
%     mat_filename = "locations"+string(nLocations)+"_creation_test.mat";
%     load(mat_filename, 'value_iteration_parameters');
%     value_iteration_time(nLocations) = value_iteration_parameters.value_iteration_time;
%     %exec_init_time(nLocations) = pre_execution_parameters.initialization_time;
% end
% %Parse CPU/mem log file;
% logID = fopen("increase_locals_creation2.txt", 'r');
% formatSpec = '%d,%f,%f,%f,%f,%f,%f';
% sizeA = [7 Inf];
% creation_cpu_mem = fscanf(logID,formatSpec, sizeA);
% nSamples = size(creation_cpu_mem, 2);
% %avg_creation_cpu_mem = zeros(7, Inf, max_nLocations);
% all_samples_creation_cpu_mem = cell(max_nLocations, 1);
% 
% for sample = 1:nSamples
%     measure = creation_cpu_mem(:,sample);
%     nLocation = creation_cpu_mem(1,sample);
%     all_samples_creation_cpu_mem{nLocation} = cat(2, all_samples_creation_cpu_mem{nLocation}, measure);
% end
% 
% avg_creation_cpu_mem = zeros(7, max_nLocations);
% for nLocation = 1:max_nLocations
%     avg_creation_cpu_mem(:,nLocation) = mean(all_samples_creation_cpu_mem{nLocation, 1}, 2);
% end
% avg_creation_cpu_mem(5,:) = avg_creation_cpu_mem(5,:)*harode_total_memory;
% avg_creation_cpu_mem(6,:) = avg_creation_cpu_mem(6,:)*harode_total_memory;
% avg_creation_cpu_mem(7,:) = avg_creation_cpu_mem(7,:)*harode_total_memory;
% 
% logID = fopen("increase_locals_log_preexecution.txt", 'r');
% formatSpec = '%d,%f,%f,%f,%f,%f,%f';
% sizeA = [7 Inf];
% preexecution_cpu_mem = fscanf(logID,formatSpec, sizeA);
% 
% nSamples = size(preexecution_cpu_mem, 2);
% %avg_creation_cpu_mem = zeros(7, Inf, max_nLocations);
% all_samples_preexecution_cpu_mem = cell(max_nLocations, 1);
% 
% for sample = 1:nSamples
%     measure = preexecution_cpu_mem(:,sample);
%     nLocation = preexecution_cpu_mem(1,sample);
%     all_samples_preexecution_cpu_mem{nLocation} = cat(2, all_samples_preexecution_cpu_mem{nLocation}, measure);
% end
% 
% avg_preexecution_cpu_mem = zeros(7, max_nLocations);
% for nLocation = 1:max_nLocations
%     avg_preexecution_cpu_mem(:,nLocation) = mean(all_samples_preexecution_cpu_mem{nLocation, 1}, 2);
% end
% avg_preexecution_cpu_mem(5,:) = avg_preexecution_cpu_mem(5,:)*harode_total_memory;
% avg_preexecution_cpu_mem(6,:) = avg_preexecution_cpu_mem(6,:)*harode_total_memory;
% avg_preexecution_cpu_mem(7,:) = avg_preexecution_cpu_mem(7,:)*harode_total_memory;
% 
% 
% logID = fopen("increase_locals_log_execution_matlab.txt", 'r');
% formatSpec = '%d,%f,%f,%f,%f,%f,%f';
% sizeA = [7 Inf];
% execution_mat_cpu_mem = fscanf(logID,formatSpec, sizeA);
% 
% nSamples = size(execution_mat_cpu_mem, 2);
% %avg_creation_cpu_mem = zeros(7, Inf, max_nLocations);
% all_samples_execution_mat_cpu_mem = cell(max_nLocations, 1);
% 
% for sample = 1:nSamples
%     measure = execution_mat_cpu_mem(:,sample);
%     nLocation = execution_mat_cpu_mem(1,sample);
%     all_samples_execution_mat_cpu_mem{nLocation} = cat(2, all_samples_execution_mat_cpu_mem{nLocation}, measure);
% end
% 
% avg_execution_mat_cpu_mem = zeros(7, max_nLocations);
% for nLocation = 1:max_nLocations
%     avg_execution_mat_cpu_mem(:,nLocation) = mean(all_samples_execution_mat_cpu_mem{nLocation, 1}, 2);
% end
% 
% 
% logID = fopen("increase_locals_log_execution_ros.txt", 'r');
% formatSpec = '%d,%f,%f,%f,%f,%f,%f';
% sizeA = [7 Inf];
% execution_ros_cpu_mem = fscanf(logID,formatSpec, sizeA);
% 
% nSamples = size(execution_ros_cpu_mem, 2);
% %avg_creation_cpu_mem = zeros(7, Inf, max_nLocations);
% all_samples_execution_ros_cpu_mem = cell(max_nLocations, 1);
% 
% for sample = 1:nSamples
%     measure = execution_ros_cpu_mem(:,sample);
%     nLocation = execution_ros_cpu_mem(1,sample);
%     all_samples_execution_ros_cpu_mem{nLocation} = cat(2, all_samples_execution_ros_cpu_mem{nLocation}, measure);
% end
% 
% avg_execution_ros_cpu_mem = zeros(7, max_nLocations);
% for nLocation = 1:max_nLocations
%     avg_execution_ros_cpu_mem(:,nLocation) = mean(all_samples_execution_ros_cpu_mem{nLocation, 1}, 2);
% end
% 
% avg_execution_total_cpu_mem = zeros(7, max_nLocations);
% for nLocation = 1:max_nLocations
%     avg_execution_total_cpu_mem(1,nLocation) = avg_execution_ros_cpu_mem(1,nLocation);
%     avg_execution_total_cpu_mem(2,nLocation) = avg_execution_mat_cpu_mem(2,nLocation)+avg_execution_ros_cpu_mem(2,nLocation);
%     avg_execution_total_cpu_mem(3,nLocation) = avg_execution_mat_cpu_mem(3,nLocation)+avg_execution_ros_cpu_mem(3,nLocation);
%     avg_execution_total_cpu_mem(4,nLocation) = avg_execution_mat_cpu_mem(4,nLocation)+avg_execution_ros_cpu_mem(4,nLocation);
%     avg_execution_total_cpu_mem(5,nLocation) = avg_execution_mat_cpu_mem(5,nLocation)+avg_execution_ros_cpu_mem(5,nLocation);
%     avg_execution_total_cpu_mem(6,nLocation) = avg_execution_mat_cpu_mem(6,nLocation)+avg_execution_ros_cpu_mem(6,nLocation);
%     avg_execution_total_cpu_mem(2,nLocation) = avg_execution_mat_cpu_mem(7,nLocation)+avg_execution_ros_cpu_mem(7,nLocation);
% end
% avg_execution_total_cpu_mem(5,:) = avg_execution_total_cpu_mem(5,:)*harode_total_memory;
% avg_execution_total_cpu_mem(6,:) = avg_execution_total_cpu_mem(6,:)*harode_total_memory;
% avg_execution_total_cpu_mem(7,:) = avg_execution_total_cpu_mem(7,:)*harode_total_memory;

%% Plotting
% close all
% plot(MDP_creation_time./60, 'b')
% hold on
% grid minor
% grid
% scatter(linspace(1,max_nLocations,max_nLocations), MDP_creation_time./60, 'red', 'x')
% xlabel('Number of locations','FontSize',12,'FontWeight','bold')
% ylabel('Time [minutes]','FontSize',12,'FontWeight','bold')
% 
% figure;
% plot(GSPNR_creation_time, 'b')
% grid minor
% grid on
% hold on
% scatter(linspace(1,max_nLocations,max_nLocations), GSPNR_creation_time, 'red', 'x')
% xlabel('Number of locations','FontSize',12,'FontWeight','bold')
% ylabel('Time [seconds]','FontSize',12,'FontWeight','bold')
% 
% figure;
% plot(nStates, 'b')
% hold on
% grid minor
% grid on
% scatter(linspace(1,max_nLocations,max_nLocations), nStates, 'red', 'x')
% xlabel('Number of locations','FontSize',12,'FontWeight','bold')
% ylabel('Number of states','FontSize',12,'FontWeight','bold')
% 
% figure;
% errorbar(avg_creation_cpu_mem(1,:), avg_creation_cpu_mem(2,:), [], avg_creation_cpu_mem(3,:)-avg_creation_cpu_mem(2,:), 'xb');
% grid minor
% grid on
% hold on
% scatter(avg_creation_cpu_mem(1,:), avg_creation_cpu_mem(2,:), 'red', 'x')
% %scatter(avg_creation_cpu_mem(1,:), avg_creation_cpu_mem(3,:), 'blue', 'x')
% xlabel('Number of locations','FontSize',12,'FontWeight','bold')
% ylabel('CPU usage [% of a single core]','FontSize',12,'FontWeight','bold')
% legend(['xr' 'xb'],{'Maximum Value', 'Mean Value'},'Location','southeast')
% 
% figure;
% errorbar(avg_creation_cpu_mem(1,:), avg_creation_cpu_mem(5,:), [], avg_creation_cpu_mem(6,:)-avg_creation_cpu_mem(5,:), 'xb');
% grid minor
% grid on
% hold on
% scatter(avg_creation_cpu_mem(1,:), avg_creation_cpu_mem(5,:), 'red', 'x')
% xlabel('Number of locations','FontSize',12,'FontWeight','bold')
% ylabel('Memory usage [GB]','FontSize',12,'FontWeight','bold')
% legend(['xr' 'b'],{'Maximum Value', 'Mean Value'},'Location','southeast')
% 
% figure;
% plot(nStates(1:39), value_iteration_time./60, 'b')
% hold on
% grid minor
% grid
% scatter(nStates(1:39), value_iteration_time./60, 'red', 'x')
% xlabel('Number of states','FontSize',12,'FontWeight','bold')
% ylabel('Time [minutes]','FontSize',12,'FontWeight','bold')
% 
% figure;
% plot(nPlaces, 'b');
% grid minor
% grid on
% hold on
% scatter(linspace(1,max_nLocations,max_nLocations) , nPlaces, 'red', 'x')
% xlabel('Number of locations','FontSize',12,'FontWeight','bold')
% ylabel('Number of places in the GSPNR Model','FontSize',12,'FontWeight','bold')
% 
% figure;
% errorbar(avg_execution_total_cpu_mem(1,:), avg_execution_total_cpu_mem(5,:), [], avg_execution_total_cpu_mem(6,:)-avg_execution_total_cpu_mem(5,:), 'xb');
% grid minor
% grid on
% hold on
% scatter(avg_execution_total_cpu_mem(1,:), avg_execution_total_cpu_mem(5,:), 'red', 'x')
% xlabel('Number of locations','FontSize',12,'FontWeight','bold')
% ylabel('Memory usage [GB]','FontSize',12,'FontWeight','bold')
% legend(['xr' 'b'],{'Maximum Value', 'Mean Value'},'Location','southeast')
% 
% figure;
% errorbar(avg_execution_total_cpu_mem(1,:), avg_execution_total_cpu_mem(2,:), [], avg_execution_total_cpu_mem(3,:)-avg_execution_total_cpu_mem(2,:), 'xb');
% grid minor
% grid on
% hold on
% scatter(avg_execution_total_cpu_mem(1,:), avg_execution_total_cpu_mem(2,:), 'red', 'x')
% %scatter(avg_creation_cpu_mem(1,:), avg_creation_cpu_mem(3,:), 'blue', 'x')
% xlabel('Number of locations','FontSize',12,'FontWeight','bold')
% ylabel('CPU usage [% of a single core]','FontSize',12,'FontWeight','bold')
% legend(['xr' 'b'],{'Maximum Value', 'Mean Value'},'Location','southeast')

  %% Parsing for increasing robots
% 
% max_nRobots = 6;
% 
% %Prepare arrays to hold data (index will be number of locations for that
% %test)
% robot_nPlaces = zeros(1, max_nRobots);
% robot_nTransitions = zeros(1, max_nRobots);
% robot_nEdges = zeros(1, max_nRobots);
% robot_GSPNR_creation_time = zeros(1, max_nRobots);
% robot_MDP_creation_time = zeros(1, max_nRobots);
% robot_nStates = zeros(1, max_nRobots);
% exec_init_time = zeros(1, max_nRobots);
% 
% %Parse all MAT files;
% for nRobots = 1:max_nRobots
%     mat_filename = "robots"+string(nRobots)+"_creation_test.mat";
%     load(mat_filename, 'parameters')%, 'pre_execution_parameters');
%     robot_nPlaces(nRobots) = parameters.nPlaces;
%     robot_nTransitions(nRobots) = parameters.nTransitions;
%     robot_nEdges(nRobots) = parameters.nEdges;
%     robot_GSPNR_creation_time(nRobots) = parameters.GSPNR_creation_time;
%     robot_MDP_creation_time(nRobots) = parameters.MDP_creation_time;
%     robot_nStates(nRobots) = parameters.nStates;
%     %exec_init_time(nRobots) = pre_execution_parameters.initialization_time;
% end
% %Parse CPU/mem log file;
% logID = fopen("increasing_robots_creation_log_cpu_mem.txt", 'r');
% formatSpec = '%d,%f,%f,%f,%f,%f,%f';
% sizeA = [7 Inf];
% robots_creation_cpu_mem = fscanf(logID,formatSpec, sizeA);
% nSamples = size(robots_creation_cpu_mem, 2);
% %avg_creation_cpu_mem = zeros(7, Inf, max_nLocations);
% all_samples_robot_creation_cpu_mem = cell(max_nRobots, 1);
% 
% for sample = 1:nSamples
%     measure = robots_creation_cpu_mem(:,sample);
%     nRobot = robots_creation_cpu_mem(1,sample);
%     all_samples_robot_creation_cpu_mem{nRobot} = cat(2, all_samples_robot_creation_cpu_mem{nRobot}, measure);
% end
% 
% avg_robot_creation_cpu_mem = zeros(7, max_nRobots);
% for nRobot = 1:max_nRobots
%     avg_robot_creation_cpu_mem(:,nRobot) = mean(all_samples_robot_creation_cpu_mem{nRobot, 1}, 2);
% end
% avg_robot_creation_cpu_mem(5,:) = avg_robot_creation_cpu_mem(5,:)*harode_total_memory;
% avg_robot_creation_cpu_mem(6,:) = avg_robot_creation_cpu_mem(6,:)*harode_total_memory;
% avg_robot_creation_cpu_mem(7,:) = avg_robot_creation_cpu_mem(7,:)*harode_total_memory;
% 
% 
% logID = fopen("increase_robots_log_execution_matlab.txt", 'r');
% formatSpec = '%d,%f,%f,%f,%f,%f,%f';
% sizeA = [7 Inf];
% robots_execution_mat_cpu_mem = fscanf(logID,formatSpec, sizeA);
% 
% nSamples = size(robots_execution_mat_cpu_mem, 2);
% %avg_creation_cpu_mem = zeros(7, Inf, max_nLocations);
% all_samples_robot_execution_mat_cpu_mem = cell(max_nRobots, 1);
% 
% for sample = 1:nSamples
%     measure = robots_execution_mat_cpu_mem(:,sample);
%     nRobot = robots_execution_mat_cpu_mem(1,sample);
%     all_samples_robot_execution_mat_cpu_mem{nRobot} = cat(2, all_samples_robot_execution_mat_cpu_mem{nRobot}, measure);
% end
% 
% avg_robot_execution_mat_cpu_mem = zeros(7, max_nRobots);
% for nRobot = 2:max_nRobots
%     avg_robot_execution_mat_cpu_mem(:,nRobot) = mean(all_samples_robot_execution_mat_cpu_mem{nRobot, 1}, 2);
% end
% 
% 
% logID = fopen("increase_robots_log_execution_ros.txt", 'r');
% formatSpec = '%d,%f,%f,%f,%f,%f,%f';
% sizeA = [7 Inf];
% robots_execution_ros_cpu_mem = fscanf(logID,formatSpec, sizeA);
% 
% nSamples = size(robots_execution_ros_cpu_mem, 2);
% %avg_creation_cpu_mem = zeros(7, Inf, max_nLocations);
% all_samples_robot_execution_ros_cpu_mem = cell(max_nRobots, 1);
% 
% for sample = 1:nSamples
%     measure = robots_execution_ros_cpu_mem(:,sample);
%     nRobot = robots_execution_ros_cpu_mem(1,sample);
%     all_samples_robot_execution_ros_cpu_mem{nRobot} = cat(2, all_samples_robot_execution_ros_cpu_mem{nRobot}, measure);
% end
% 
% avg_robot_execution_ros_cpu_mem = zeros(7, max_nRobots);
% for nRobot = 2:max_nRobots
%     avg_robot_execution_ros_cpu_mem(:,nRobot) = mean(all_samples_robot_execution_ros_cpu_mem{nRobot, 1}, 2);
% end
% 
% avg_robot_execution_total_cpu_mem = zeros(7, max_nRobots);
% for nRobot = 2:max_nRobots
%     avg_robot_execution_total_cpu_mem(1,nRobot) = avg_robot_execution_ros_cpu_mem(1,nRobot);
%     avg_robot_execution_total_cpu_mem(2,nRobot) = avg_robot_execution_mat_cpu_mem(2,nRobot)+avg_robot_execution_ros_cpu_mem(2,nRobot);
%     avg_robot_execution_total_cpu_mem(3,nRobot) = avg_robot_execution_mat_cpu_mem(3,nRobot)+avg_robot_execution_ros_cpu_mem(3,nRobot);
%     avg_robot_execution_total_cpu_mem(4,nRobot) = avg_robot_execution_mat_cpu_mem(4,nRobot)+avg_robot_execution_ros_cpu_mem(4,nRobot);
%     avg_robot_execution_total_cpu_mem(5,nRobot) = avg_robot_execution_mat_cpu_mem(5,nRobot)+avg_robot_execution_ros_cpu_mem(5,nRobot);
%     avg_robot_execution_total_cpu_mem(6,nRobot) = avg_robot_execution_mat_cpu_mem(6,nRobot)+avg_robot_execution_ros_cpu_mem(6,nRobot);
%     avg_robot_execution_total_cpu_mem(2,nRobot) = avg_robot_execution_mat_cpu_mem(7,nRobot)+avg_robot_execution_ros_cpu_mem(7,nRobot);
% end
% avg_robot_execution_total_cpu_mem(5,:) = avg_robot_execution_total_cpu_mem(5,:)*harode_total_memory;
% avg_robot_execution_total_cpu_mem(6,:) = avg_robot_execution_total_cpu_mem(6,:)*harode_total_memory;
% avg_robot_execution_total_cpu_mem(7,:) = avg_robot_execution_total_cpu_mem(7,:)*harode_total_memory;
% 
 %% Plotting
% close all
% semilogy(robot_MDP_creation_time./60, 'b')
% hold on
% grid on
% grid minor
% scatter(linspace(1,max_nRobots,max_nRobots), robot_MDP_creation_time./60, 'red', 'x')
% xlabel('Number of robots','FontSize',12,'FontWeight','bold')
% ylabel('Time [minutes]','FontSize',12,'FontWeight','bold')
% 
% figure;
% plot(robot_GSPNR_creation_time, 'b')
% grid minor
% grid on
% hold on
% scatter(linspace(1,max_nRobots,max_nRobots), robot_GSPNR_creation_time, 'red', 'x')
% xlabel('Number of robots','FontSize',12,'FontWeight','bold')
% ylabel('Time [seconds]','FontSize',12,'FontWeight','bold')
% 
% figure;
% semilogy(robot_nStates, 'b')
% hold on
% grid minor
% grid on
% scatter(linspace(1,max_nRobots,max_nRobots), robot_nStates, 'red', 'x')
% xlabel('Number of robots','FontSize',12,'FontWeight','bold')
% ylabel('Number of states','FontSize',12,'FontWeight','bold')
% 
% figure;
% errorbar(avg_robot_creation_cpu_mem(1,:), avg_robot_creation_cpu_mem(2,:), [], avg_robot_creation_cpu_mem(3,:)-avg_robot_creation_cpu_mem(2,:), 'xb');
% grid minor
% grid on
% hold on
% scatter(avg_robot_creation_cpu_mem(1,:), avg_robot_creation_cpu_mem(2,:), 'red', 'x')
% %scatter(avg_creation_cpu_mem(1,:), avg_creation_cpu_mem(3,:), 'blue', 'x')
% xlabel('Number of robots','FontSize',12,'FontWeight','bold')
% ylabel('CPU usage [% of a single core]','FontSize',12,'FontWeight','bold')
% legend(['xr' 'xb'],{'Maximum Value', 'Mean Value'},'Location','southeast')
% 
% figure;
% errorbar(avg_robot_creation_cpu_mem(1,:), avg_robot_creation_cpu_mem(5,:), [], avg_robot_creation_cpu_mem(6,:)-avg_robot_creation_cpu_mem(5,:), 'xb');
% grid minor
% grid on
% hold on
% scatter(avg_robot_creation_cpu_mem(1,:), avg_robot_creation_cpu_mem(5,:), 'red', 'x')
% xlabel('Number of robots','FontSize',12,'FontWeight','bold')
% ylabel('Memory usage [GB]','FontSize',12,'FontWeight','bold')
% legend(['xr' 'b'],{'Maximum Value', 'Mean Value'},'Location','southeast')
% 
% figure;
% errorbar(avg_robot_execution_total_cpu_mem(1,:), avg_robot_execution_total_cpu_mem(2,:), [], avg_robot_execution_total_cpu_mem(3,:)-avg_robot_execution_total_cpu_mem(2,:), 'xb');
% grid minor
% grid on
% hold on
% scatter(avg_robot_execution_total_cpu_mem(1,:), avg_robot_execution_total_cpu_mem(2,:), 'red', 'x')
% %scatter(avg_creation_cpu_mem(1,:), avg_creation_cpu_mem(3,:), 'blue', 'x')
% xlabel('Number of robots','FontSize',12,'FontWeight','bold')
% ylabel('CPU usage [% of a single core]','FontSize',12,'FontWeight','bold')
% legend(['xr' 'xb'],{'Maximum Value', 'Mean Value'},'Location','southeast')
% 
% figure;
% errorbar(avg_robot_execution_total_cpu_mem(1,:), avg_robot_execution_total_cpu_mem(5,:), [], avg_robot_execution_total_cpu_mem(6,:)-avg_robot_execution_total_cpu_mem(5,:), 'xb');
% grid minor
% grid on
% hold on
% scatter(avg_robot_execution_total_cpu_mem(1,:), avg_robot_execution_total_cpu_mem(5,:), 'red', 'x')
% xlabel('Number of robots','FontSize',12,'FontWeight','bold')
% ylabel('Memory usage [GB]','FontSize',12,'FontWeight','bold')
% legend(['xr' 'b'],{'Maximum Value', 'Mean Value'},'Location','southeast')

