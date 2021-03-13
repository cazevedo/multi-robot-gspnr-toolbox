clc
clear

addpath('/home/antonio/Repos/multi-robot-gspnr-toolbox/src/models/');

test = MDP();

test.add_state("S1"); 
test.add_state("S2");

test.add_action("a","imm");
test.add_action("b","imm");

test.add_state("S3");

test

test.set_transition("S1","a","S1", 0.2, "imm")
test.set_transition("S1","a","S2", 0.5, "imm")
test.set_transition("S1","a","S3", 0.3, "imm")
test.set_transition("S1","b","S1",1.0, "imm")
test.set_reward("S1", "a", 5);
 

% 
test.check_validity()

full_trans = test.get_full_transition_matrix()
full_reward = test.get_full_reward_matrix()

test.set_enabled_actions()
test.actions_enabled("S1")
% 
% [end_state_indices, end_state_prob] = test.action_probs(1,1)

