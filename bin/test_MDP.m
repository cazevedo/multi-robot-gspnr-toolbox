clc
clear

addpath('/home/antonio/Repos/GSPNR_Toolbox/src/models/');
import MarkovDecisionProblem.MarkovDecisionProblem.*

test = MarkovDecisionProblem();

test.add_state("S1"); 
test.add_state("S2");

test.add_action("a","imm");
test.add_action("b","imm");

test.add_state("S3");

test

test.set_transition("S1","a","S2", 1.0, "imm")
test.set_reward("S1", "a", 5);

[full_trans, full_reward] = test.get_full_matrices()

