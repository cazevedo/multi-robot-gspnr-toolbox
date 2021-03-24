function gspn = ImportfromPIPE(xml_filepath)
%ImportfromPIPE Given an xml file exported from PIPE software, creates
%equivalent instance of a GSPNR object.
%Attention! Cannot import initial marking, or arc weights, and so these
%attributes must be set after generating the GSPNR object.

gspn = GSPNR();
struct = xml2struct(xml_filepath)

nPlaces = size(struct.pnml.net.place, 2)

places = [string.empty];
tokens = [];

for index = 1:nPlaces
    name = struct.pnml.net.place{1,index}.Attributes.id;
    places = cat(2, places, name);
    token_string = struct.pnml.net.place{1,index}.initialMarking.value.Text;
    token_string = erase(token_string,"Default,");
    ntokens = str2double(token_string);
    tokens = cat(2, tokens, ntokens);
end

gspn.add_places(places,tokens);

transition_names = [string.empty];
transition_types = [string.empty];
transition_rates = [];

nTrans = size(struct.pnml.net.transition, 2)

for index = 1:nTrans
    name = struct.pnml.net.transition{1,index}.Attributes.id;
    if struct.pnml.net.transition{1,index}.timed.value.Text == "true"
        %Exponential transition
        type = "exp";
    else
        %Immediate transition
        type = "imm";
        
    end
    rate = str2double(struct.pnml.net.transition{1,index}.rate.value.Text);
    transition_names = cat(2,transition_names, name);
    transition_types = cat(2,transition_types, type);
    transition_rates = cat(2,transition_rates, rate);
end

gspn.add_transitions(transition_names,transition_types, transition_rates);

arc_places = [string.empty];
arc_trans = [string.empty];
arc_type = [string.empty];
arc_weight = [];

nArcs = size(struct.pnml.net.arc,2)

for index = 1:nArcs
    source = struct.pnml.net.arc{1,index}.Attributes.source;
    target = struct.pnml.net.arc{1,index}.Attributes.target;
    %Checking if source node is a place:
    if isempty(find(ismember(places, source)))
        %Source is a transition and arc is an output arc
        arc_places = cat(2, arc_places, target);
        arc_trans = cat(2, arc_trans, source);
        arc_type = cat(2, arc_type, "out");
        weight_string = struct.pnml.net.arc{1,index}.inscription.value.Text;
        weight = str2double(erase(weight_string,"Default,"));
        arc_weight = cat(2, arc_weight, weight);
    else
        %Source is a place and arc is an input arc
        arc_places = cat(2, arc_places, source);
        arc_trans = cat(2, arc_trans, target);
        arc_type = cat(2, arc_type, "in");
        weight_string = struct.pnml.net.arc{1,index}.inscription.value.Text;
        weight = str2double(erase(weight_string,"Default,"));
        arc_weight = cat(2, arc_weight, weight);
    end    
end

gspn.add_arcs(arc_places, arc_trans, arc_type, arc_weight);

end

