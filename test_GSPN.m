clear
clc

places = ["p1", "p2", "p3", "p4"];
tokens = [0, 1, 1, 0];

transitions = ["t1", "t2", "t3"];
types = ["imm", "exp", "imm"];
rates = [1, 1, 1];

%transitions_2 = ["t4"];
%types_2 = ["imm"];

arc_places =        ["p1", "p2", "p3", "p2", "p3", "p4", "p4", "p1"];
arc_transitions =   ["t1", "t1", "t1", "t2", "t2", "t2", "t3", "t3"];
arc_type =          ["in", "out", "out", "in", "in", "out", "in", "out"];
arc_weight =        [1, 1, 1, 1, 1, 1, 1, 1];

reward_names = ["p3", "t3"];
reward_values = [1, 5];
reward_types = ["place", "transition"];

test = GSPNR();
test.add_places(places, tokens)
test.add_transitions(transitions, types, rates)
test.places;
test.initial_marking;
test.transitions;
test.add_arcs(arc_places, arc_transitions, arc_type, arc_weight)
test.set_reward_functions(reward_names, reward_values, reward_types)


%test.add_transitions(transitions_2, types_2)
%test.transitions;
[imm_transitions, exp_transitions] = test.enabled_transitions()

%test.fire_transition("t2")

test
