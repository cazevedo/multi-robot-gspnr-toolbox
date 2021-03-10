clc
clear

addpath('/home/antonio/Repos/multi-robot-gspnr-toolbox/src/models/');
import MarkovDecisionProblem.MarkovDecisionProblem.*

test = MarkovDecisionProblem();

test.add_state("S1"); 
test.add_state("S2");

test.add_action("a","imm");
test.add_action("b","imm");

test.add_state("S3");

test

test.set_transition("S1","a","S1", 0.2, "imm")
test.set_transition("S1","a","S2", 0.5, "imm")
test.set_transition("S1","a","S3", 0.3, "imm")
test.set_reward("S1", "a", 5);

[full_trans, full_reward] = test.get_full_matrices()

[validity, cum_prob] = test.check_validity()

[end_state_indices, end_state_prob] = test.action_probs(1,1)

