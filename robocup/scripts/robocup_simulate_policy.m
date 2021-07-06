clc
clear

load("robocup_last_policy.mat");

policy_struct.gspn = homeModel;
policy_struct.state_index_to_markings = markings;
policy_struct.states = states;
policy_struct.mdp = mdp;
policy_struct.mdp_policy = policy;

results = homeModel.evaluate_policy(policy_struct, 40000);