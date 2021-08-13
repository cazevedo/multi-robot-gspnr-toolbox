clc
clear

load("comparing_memory.mat", "results");

nRobots = results(:,1);
nStates = results(:,2);
%Converting memory measured from kB to MB
gspn_mem = results(:,3)./(1024);
mdp_mem = results(:,4)./(1024);

figure;

p = plot(nRobots, gspn_mem, nRobots, mdp_mem);
p(1).Color = [0 0 1];
p(2).Color = [0.8500 0.3250 0.0980];

hold on;

scatter(nRobots, gspn_mem, 'red', 'x');
scatter(nRobots, mdp_mem, 'red', 'x');

grid on;
grid minor;

xlabel('Number of robots','FontSize',12,'FontWeight','bold')
ylabel('Memory usage [MB]','FontSize',12,'FontWeight','bold')
legend(['xr' 'xb'],{'GSPNR', 'MDP'},'Location','northwest')

figure;

p = plot(nStates, gspn_mem, nStates, mdp_mem);
p(1).Color = [0 0 1];
p(2).Color = [0.8500 0.3250 0.0980];

hold on;

scatter(nStates, gspn_mem, 'red', 'x');
scatter(nStates, mdp_mem, 'red', 'x');

grid on;
grid minor;

xlabel('Number of states','FontSize',12,'FontWeight','bold')
ylabel('Memory usage [MB]','FontSize',12,'FontWeight','bold')
legend(['xr' 'xb'],{'GSPNR', 'MDP'},'Location','northwest')