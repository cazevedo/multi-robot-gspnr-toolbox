clc
clear

addpath('/home/antonio/Repos/multi-robot-gspnr-toolbox/src/models/');

import GSPNR.*
import MarkovDecisionProcess.MarkovDecisionProcess.*

% deterministic = GSPNR();
% 
% places = ["p1", "p2", "p3", "p4", "p5", "p6"];
% tokens = [1,    0,    0,    0,    0,    0   ];
% 
% transitions =    ["t1", "t2", "t3", "t4", "t5"];
% transition_types=["imm","i600 ++    872 --mm","imm","imm","imm"];
% transition_rates=[0,    0,    0,    1,    2   ];
% 
% arc_places = ["p1","p1","p1","p2", "p2","p2","p3", "p4", "p5", "p6"];
% arc_trans  = ["t1","t2","t3","t1", "t4","t5","t2", "t3", "t4", "t5"];
% arc_type   = ["in","in","in","out","in","in","out","out","out","out"];
% arc_weights= [1   ,1   ,1   ,1   ,1   ,1   ,1   ,1   ,1   ,1];
% 
% deterministic.add_places(places,tokens);
% deterministic.add_transitions(transitions,transition_types,transition_rates);
% deterministic.add_arcs(arc_places,arc_trans,arc_type,arc_weights);
% 
% MDP = deterministic.ConverttoMDP();

small_hybrid = GSPNR();
places = ["p1", "p2", "p3", "p4"];
tokens = [2   , 0   , 0   , 0   ];

transitions =       ["t1", "t2", "t3"];
transition_types =  ["imm","imm","exp"];
transition_rates =  [0,    0,    1   ];

arc_places = ["p1", "p1", "p1", "p2", "p3", "p4"];
arc_trans  = ["t1", "t2", "t3", "t1", "t2", "t3"];
arc_type =   ["in", "in", "in", "out","out","out"];
arc_weights= [1   , 1   , 1   , 1    , 1   ,1   ];

reward_names = ["p1", "t1"];
reward_types = ["place", "transition"];
reward_values= [5, 3];

small_hybrid.add_places(places,tokens);
small_hybrid.add_transitions(transitions, transition_types, transition_rates);
small_hybrid.add_arcs(arc_places,arc_trans,arc_type,arc_weights);
small_hybrid.set_reward_functions(reward_names, reward_values, reward_types);

[MDP, markings, states, types] = small_hybrid.toMDP()
validity = MDP.check_validity()

MDP.actions_enabled("S1")

% test_norm = GSPNR();
% places = ["p1", "p2", "p3", "p4", "p5"];
% tokens = [2   , 0   , 0   , 0   , 0];
% 
% transitions =    ["t1", "t2", "t3", "t4", "t5", "t6"];
% transition_types=["exp","exp","exp","exp","exp","exp"];
% transition_rates=[1,    1,    2,    1.5,   3   , 1.5];
% 
% arc_places = ["p1","p1","p1","p2" , "p3","p4" ,"p4" , "p4", "p4", "p5", "p4", "p5"];
% arc_trans  = ["t1","t2","t3","t1" , "t2","t3" ,"t4" , "t5", "t5", "t4", "t6", "t6"];
% arc_type   = ["in","in","in","out","out","out","in" ,"in" ,"out","out", "in", "out"];
% arc_weights= [1   ,1   ,1   ,1    ,1    ,1    ,1    ,1    ,1    ,1    , 1   , 1];
% test_norm.add_places(places,tokens);
% test_norm.add_transitions(transitions,transition_types,transition_rates);
% test_norm.add_arcs(arc_places,arc_trans,arc_type,arc_weights);
% 
% [MDP, markings, states, types] = test_norm.toMDP()
% 
% validity = MDP.check_validity()

[full_transition, full_rewards] = MDP.get_full_matrices();