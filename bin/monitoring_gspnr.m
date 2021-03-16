clear
clc

%% Instantiate GSPNR
monitor_gspnr = GSPNR();

%% Add Places
places = ["MonitoringL1", "MonitoringL2", "ReadyL1", "ReadyL2", "Charging", "NavigatingL1ToL2", "NavigatingL2ToL1"]
tokens = [1, 1, 0, 0, 1, 0, 0];

monitor_gspnr.add_places(places, tokens);


%% Add transitions
transitions = ["GoFromL1ToL2", "GoFromL2ToL1", "RepeatL1", "RepeatL2", "ArrivedL1", "ArrivedL2", "FinishedL1", "FinishedL2", "ChargedL1", "ChargedL2", "DischargedL1", "DischargedL2"];
types = ["imm", "imm", "imm", "imm", "exp", "exp", "exp", "exp", "exp", "exp", "exp", "exp"];
rates = [0, 0, 0, 0, 0.5, 0.5, 8, 8, 0.01, 0.01, 0.01, 0.01];

monitor_gspnr.add_transitions(transitions, types, rates);


%% Add Arcs
arc_places =        ["MonitoringL1","MonitoringL1","MonitoringL1",...
                     "MonitoringL2","MonitoringL2","MonitoringL2",...
                     "ReadyL1","ReadyL1","ReadyL1","ReadyL1",...
                     "ReadyL2","ReadyL2","ReadyL2","ReadyL2",...
                     "NavigatingL1ToL2","NavigatingL1ToL2","NavigatingL1ToL2",...
                     "NavigatingL2ToL1","NavigatingL2ToL1","NavigatingL2ToL1",...
                     "Charging","Charging","Charging","Charging"];  
arc_transitions =   ["FinishedL1", "RepeatL1", "ArrivedL1",...
                     "FinishedL2", "RepeatL2", "ArrivedL2",...
                     "ChargedL1","FinishedL1","RepeatL1","GoFromL1ToL2",...
                     "ChargedL2","FinishedL2","RepeatL2","GoFromL2ToL1",...
                     "GoFromL1ToL2", "ArrivedL2", "DischargedL1",...
                     "GoFromL2ToL1", "ArrivedL1", "DischargedL2",...
                     "DischargedL1", "DischargedL2", "ChargedL1", "ChargedL2"];
arc_type =          ["in", "out", "out",...
                     "in", "out", "out",...
                     "out", "out", "in", "in",...
                     "out", "out", "in", "in",...
                     "out", "in", "in",...
                     "out", "in", "in",...
                     "out", "out", "in", "in"];
arc_weights = ones(1,length(arc_places));

monitor_gspnr.add_arcs(arc_places, arc_transitions, arc_type, arc_weights);

%% Print Initial Marking
monitor_gspnr.current_marking
 
%% Print Enabled Transitions
% disp("Enabled transitions")
[imm_enabled, exp_enabled] = monitor_gspnr.enabled_transitions()

%% Fire Transition
monitor_gspnr.fire_transition("FinishedL1")
monitor_gspnr.current_marking

%% Print Enabled Transitions
[imm_enabled, exp_enabled] = monitor_gspnr.enabled_transitions()

%% Fire Transition
monitor_gspnr.fire_transition("GoFromL1ToL2")
monitor_gspnr.current_marking

