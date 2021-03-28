function gspn = ImportfromPIPE(xml_filepath)
%ImportfromPIPE Given an xml file with PNML structure exported from PIPE software, creates
%equivalent instance of a GSPNR object.

gspn = GSPNR();
struct = readstruct(xml_filepath);

nPlaces = size(struct.net.place, 2);

places = [string.empty];
tokens = [];

for index = 1:nPlaces
    name = struct.net.place(index).name.value;
    places = cat(2, places, name);
    token_string = struct.net.place(index).initialMarking.value;
    token_string = erase(token_string,"Default,");
    ntokens = str2double(token_string);
    tokens = cat(2, tokens, ntokens);
end

gspn.add_places(places,tokens);

transition_names = [string.empty];
transition_types = [string.empty];
transition_rates = [];

nTrans = size(struct.net.transition, 2);

for index = 1:nTrans
    name = struct.net.transition(index).name.value;
    if struct.net.transition(index).timed.value == "true"
        %Exponential transition
        type = "exp";
    else
        %Immediate transition
        type = "imm";
        
    end
    rate = struct.net.transition(index).rate.value;
    transition_names = cat(2,transition_names, name);
    transition_types = cat(2,transition_types, type);
    transition_rates = cat(2,transition_rates, rate);
end

gspn.add_transitions(transition_names,transition_types, transition_rates);

arc_places = [string.empty];
arc_trans = [string.empty];
arc_type = [string.empty];
arc_weight = [];

nArcs = size(struct.net.arc,2);

for index = 1:nArcs
    source = struct.net.arc(index).sourceAttribute;
    target = struct.net.arc(index).targetAttribute;
    %Checking if source node is a place:
    if isempty(find(ismember(places, source)))
        %Source is a transition and arc is an output arc
        arc_places = cat(2, arc_places, target);
        arc_trans = cat(2, arc_trans, source);
        arc_type = cat(2, arc_type, "out");
        weight_string = struct.net.arc(index).inscription.value;
        weight = str2double(erase(weight_string,"Default,"));
        arc_weight = cat(2, arc_weight, weight);
    else
        %Source is a place and arc is an input arc
        arc_places = cat(2, arc_places, source);
        arc_trans = cat(2, arc_trans, target);
        arc_type = cat(2, arc_type, "in");
        weight_string = struct.net.arc(index).inscription.value;
        weight = str2double(erase(weight_string,"Default,"));
        arc_weight = cat(2, arc_weight, weight);
    end    
end

gspn.add_arcs(arc_places, arc_trans, arc_type, arc_weight);

end

