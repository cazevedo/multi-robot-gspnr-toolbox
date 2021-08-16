clc
clear

load('/home/antonio/Repos/gspnr_matlab/multi-robot-gspnr-toolbox/examples/home_GSPNR.mat')

gspn = home_GSPNR;
nPlaces = size(gspn.places, 2);
marking = zeros(1, nPlaces);

initial_place_index = gspn.find_place_index("L1");

done = 0;
nRobots = 0;

results = zeros(1, 4);

while ~done
    nRobots = nRobots + 1;
    new_marking = marking;
    new_marking(initial_place_index) = nRobots;
    
    gspn.set_initial_marking(new_marking);
    
    results(nRobots, 1) = nRobots;
    results(nRobots, 3) = gspn.measure_memory();
    
    disp(["Started converting to MDP with ", nRobots, " at time - ",string(datetime('now'))])
    
    [mdp, ~, ~, ~] = gspn.toMDP_without_wait();
    
    results(nRobots, 2) = mdp.measure_memory();
    results(nRobots, 4) = mdp.nStates;

end