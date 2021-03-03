clc
clear

deterministic = GSPNR();

places = ["p1", "p2", "p3", "p4", "p5", "p6"];
tokens = [2,    0,    0,    0,    0,    0   ];

transitions =    ["t1", "t2", "t3", "t4", "t5"];
transition_types=["imm","imm","imm","imm","imm"];
transition_rates=[0,    0,    0,    1,    1   ];

arc_places = ["p1","p1","p1","p2", "p2","p2","p3", "p4", "p5", "p6"];
arc_trans  = ["t1","t2","t3","t1", "t4","t5","t2", "t3", "t4", "t5"];
arc_type   = ["in","in","in","out","in","in","out","out","out","out"];
arc_weights= [1   ,1   ,1   ,1   ,1   ,1   ,1   ,1   ,1   ,1];

deterministic.add_places(places,tokens);
deterministic.add_transitions(transitions,transition_types,transition_rates);
deterministic.add_arcs(arc_places,arc_trans,arc_type,arc_weights);

MDP = deterministic.ConverttoMDP()