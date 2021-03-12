clc
clear

addpath('/home/antonio/Repos/multi-robot-gspnr-toolbox/src/models/');
addpath('/home/antonio/Repos/multi-robot-gspnr-toolbox/src/solvers/');

gridworld = MDP();

gridworld.add_state("S1");
gridworld.add_state("S2");
gridworld.add_state("S3");
gridworld.add_state("S4");
gridworld.add_state("S5");
gridworld.add_state("S6");
gridworld.add_state("S7");
gridworld.add_state("S8");
gridworld.add_state("S9");
gridworld.add_state("S10");
gridworld.add_state("S11");
gridworld.add_state("S12");
gridworld.add_state("S13");
gridworld.add_state("S14");
gridworld.add_state("S15");
gridworld.add_state("S16");
gridworld.add_state("terminal1");
gridworld.add_state("terminal2");

gridworld.add_action("down", "imm");
gridworld.add_action("up", "imm");
gridworld.add_action("left", "imm");
gridworld.add_action("right", "imm");
gridworld.add_action("finished", "imm");
gridworld.add_action("self-loop", "imm");

nStates = 16;

for n = 1:nStates
    state_name = "S"+string(n);
    
    if mod(n,4) ~= 0
        %not at right edge
        end_state_name = "S"+string(n+1);
        gridworld.set_transition(state_name, 'right', end_state_name, 1.0, "imm");
    end
    if mod(n-1,4) ~= 0
        %not at left edge
        end_state_name = "S"+string(n-1);
        gridworld.set_transition(state_name, 'left', end_state_name, 1.0, "imm");
    end
    if fix((n-1)/4) ~= 0
        %not at bottom edge
        end_state_name = "S"+string(n-4);
        gridworld.set_transition(state_name, 'down', end_state_name, 1.0, "imm");
    end
    if fix((n-1)/4) ~= 3
        %not at top edge
        end_state_name = "S"+string(n+4);
        gridworld.set_transition(state_name, 'up', end_state_name, 1.0, "imm");
    end
end

gridworld.set_transition("S16", "finished", "terminal1", 1.0, "imm")
gridworld.set_transition("S13", "finished", "terminal2", 1.0, "imm")
%gridworld.set_transition("S13", "finished", "S13", 1.0, "imm")

gridworld.set_transition("terminal1", "self-loop", "terminal1", 1.0, "imm")
gridworld.set_transition("terminal2", "self-loop", "terminal2", 1.0, "imm")

gridworld.set_reward("S16", 'finished', 10);
gridworld.set_reward("S13", 'finished', 100);
%gridworld.set_reward("S8", 'up', -100);
%gridworld.set_reward("S11", 'right', -100);
gridworld.check_validity();
gridworld.set_enabled_actions();
[values, policy] = value_iteration(gridworld, 1, 1, 0.01)
