clear
clc

places = ["L1", "L2", "L3", "NavigatingL1-L3", "NavigatingL3-L1", "NavigatingL1-L2", "NavigatingL2-L1"];
tokens = [2, 0, 0, 0, 0, 0, 0];

transitions = ["GoL1-L3", "GoL3-L1", "GoL1-L2", "GoL2-L1", "ArrivedL1-L3", "ArrivedL3-L1", "ArrivedL1-L2", "ArrivedL2-L1"];
types = ["imm", "imm", "imm", "imm", "exp", "exp", "exp", "exp"];
rates = [1, 1, 1, 1, 1, 1, 1, 1];

arc_places =        ["L1",     "NavigatingL1-L3","NavigatingL1-L3","ArrivedL1-L3","L3",     "NavigatingL3-L1","NavigatingL3-L1","L1",          "L1",     "NavigatingL1-L2","NavigatingL1-L2","ArrivedL1-L2","L2",     "NavigatingL2-L1","NavigatingL2-L1","L1"];    
arc_transitions =   ["GoL1-L3","GoL1-L3",        "ArrivedL1-L3",   "L3",          "GoL3-L1","GoL3-L1",        "ArrivedL3-L1",   "ArrivedL3-L1","GoL1-L2","GoL1-L2",        "ArrivedL1-L2",   "L2",          "GoL2-L1","GoL2-L1",        "ArrivedL2-L1",   "ArrivedL2-L1"];
arc_type =          ["in",     "out",            "in",             "out",         "in",     "out",            "in",             "out",         "in",     "out",            "in",             "out"          "in",     "out",            "in",             "out"];
arc_weights =       [1,        1,                1,                1,             1,        1,                1,                1,             1,        1,                1,                1,             1,        1,                1,                1];


proposal = GSPNR();
proposal.add_places(places, tokens);
proposal.add_transitions(transitions, types, rates);
proposal.add_arcs(arc_places, arc_transitions, arc_type, arc_weights);

proposal.enabled_transitions()
proposal.fire_transition("GoL1-L3")
proposal.current_marking
[imm_enabled, exp_enabled] = proposal.enabled_transitions()
proposal.fire_transition("GoL1-L2")
[imm_enabled, exp_enabled] = proposal.enabled_transitions()