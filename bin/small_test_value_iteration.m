clc
clear

addpath('/home/antonio/Repos/multi-robot-gspnr-toolbox/src/models/');
addpath('/home/antonio/Repos/multi-robot-gspnr-toolbox/src/solvers/');
import MarkovDecisionProblem.MarkovDecisionProblem.*
import value_iteration.value_iteration.*

mdp = MarkovDecisionProblem();

mdp.add_state("S1");
mdp.add_state("S2");
mdp.add_state("S3");
mdp.add_state("S4");

mdp.add_action("a", "imm");
mdp.add_action("b", "imm");

mdp.set_transition("S1", "a", "S3", 1.0, "imm");
mdp.set_transition("S1", "b", "S2", 1.0, "imm");
mdp.set_transition("S2", "b", "S4", 1.0, "imm");

mdp.set_reward("S1","a", 10);
mdp.set_reward("S2","b", 100);

max = 1;
[values, policy] = value_iteration(mdp, max, 1, 0.01) 