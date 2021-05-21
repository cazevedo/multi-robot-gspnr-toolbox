function [parameters] = test_value_iteration(nLocations)
    parameters = struct();
    %Importing GSPNR Model from previous run test
    mat_filename = "locations"+string(nLocations)+"_creation_test.mat";
    load(mat_filename, 'GSPNR');
    
    reward_nlocation = ceil(nLocations/2);
    reward_trans_name = "Mop_L"+reward_nlocation;
    GSPNR.set_reward_functions([reward_trans_name], [1], ["transition"]);
    
    tic
    [emb_MDP, covered_marking_list, covered_state_list, covered_state_type] = GSPNR.toMDP();
    parameters.MDP_creation_time = toc;
    emb_MDP.check_validity();
    emb_MDP.set_enabled_actions();
    tic
    [values, policy] = value_iteration(emb_MDP, 1, 0.99, 0.01);
    parameters.value_iteration_time = toc;
end

