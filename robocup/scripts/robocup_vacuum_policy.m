clc
clear

load("model_with_real_rates.mat");

mdp.check_validity();
mdp.set_enabled_actions();
disp("MDP valid and set of enabled actions built")
disp("Starting value iteration")
timeout = 0;
discount_factor = 0.99;
[values, policy, error] = value_iteration(mdp, 1, discount_factor, 0.01, timeout);
disp("Finished value iteration");