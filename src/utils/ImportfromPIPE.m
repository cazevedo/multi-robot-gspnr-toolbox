function gspn = ImportfromPIPE(xml_filepath)
%ImportfromPIPE Given an xml file exported from PIPE software, creates
%equivalent instance of a GSPNR object.
%Attention! Cannot import initial marking, or arc weights, and so these
%attributes must be set after generating the GSPNR object.

gspn = GSPNR();
struct = xml2struct(xml_filepath)

nPlaces = size(struct.net.place, 2)

places = [string.empty];
tokens = zeros(1, nPlaces);

for index = 1:nPlaces
    name = struct.net.place{1,index}.Attributes.id;
    places = cat(2, places, name);
end

gspn.add_places(places,tokens);

transition_names = [string.empty];
transition_types = [string.empty];
transition_rates = [];

nExpTrans = size(struct.net.exponentialTransition, 2)

for index = 1:nExpTrans
    name = struct.net.exponentialTransition{1,index}.Attributes.id;
    type = "exp";
    rate = struct.net.exponentialTransition{1,index}.Attributes.delay;
    rate = str2double(rate);
    rate = 1/rate;
    transition_names = cat(2,transition_names, name);
    transition_types = cat(2,transition_types, type);
    transition_rates = cat(2,transition_rates, rate);
end

nImmTrans = size(struct.net.immediateTransition, 2)

for index = 1:nImmTrans
    name = struct.net.immediateTransition{1,index}.Attributes.id;
    type = "imm";
    rate = struct.net.immediateTransition{1,index}.Attributes.weight;
    rate = str2double(rate);
    transition_names = cat(2,transition_names, name);
    transition_types = cat(2,transition_types, type);
    transition_rates = cat(2,transition_rates, rate);
end

gspn.add_transitions(transition_names,transition_types, transition_rates);

arc_places = [string.empty];
arc_trans = [string.empty];
arc_type = [string.empty];

nArcs = size(struct.net.arc,2)

arc_weight = ones(1, nArcs);

for index = 1:nArcs
    source = struct.net.arc{1,index}.Attributes.fromNode;
    target = struct.net.arc{1,index}.Attributes.toNode;
    %Checking if source node is a place:
    if isempty(find(ismember(places, source)))
        %Source is a transition and arc is an output arc
        arc_places = cat(2, arc_places, target);
        arc_trans = cat(2, arc_trans, source);
        arc_type = cat(2, arc_type, "out");
    else
        %Source is a place and arc is an input arc
        arc_places = cat(2, arc_places, source);
        arc_trans = cat(2, arc_trans, target);
        arc_type = cat(2, arc_type, "in");        
    end    
end

gspn.add_arcs(arc_places, arc_trans, arc_type, arc_weight);

end

