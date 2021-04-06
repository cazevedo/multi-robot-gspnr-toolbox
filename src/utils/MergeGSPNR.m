function merged = MergeGSPNR(gspn1,gspn2)
%MERGEGSPNR Merges to two GSPNR objects
%Merges two GSPNRs by joining duplicate places and transitions.
%Ignores arc weights, sets all arc weights to 1, as to not have to solve
%conflicts between arc weights of the same arc in both GSPNRs

    merged = GSPNR();
    
    places1 = [string.empty];
    places2 = [string.empty];
    trans1 = [string.empty];
    trans2 = [string.empty];
    
    places1 = cat(2, places1, gspn1.places);
    places2 = cat(2, places2, gspn2.places);
    
    trans1 = cat(2, trans1, gspn1.transitions);
    trans2 = cat(2, trans2, gspn2.transitions);
    
    [common_places, p_indices1, p_indices2] = intersect(places1, places2);
    [common_trans,  t_indices1, t_indices2] = intersect(trans1, trans2);
    
    common_trans_types = [string.empty];
    common_trans_rates = [];
    
    
    %Save information of common places/trans so that we do not try to add
    %empty arrays after
    nCommonPlace = length(common_places);
    if nCommonPlace == 0
        has_common_places = false;
    else
        has_common_places = true;
    end
    nCommonTrans = length(common_trans);
    if nCommonTrans == 0
        has_common_trans = false;
    else
        has_common_trans = true;
    end
    
    %Perform sanity checks on transitions - Check if transitions common to
    %both GSPNRs have same type and same weight/rate;
    for index = 1:nCommonTrans
        trans_name = common_trans(index);
        t_index1 = t_indices1(index);
        t_index2 = t_indices2(index);
        
        if gspn1.type_transitions(t_index1) ~= gspn2.type_transitions(t_index2)
            error("Common transitions between the two GSPNs must be the same type");
        elseif gspn1.rate_transitions(t_index1) ~= gspn2.rate_transitions(t_index2)
            error("Common transitions between the two GSPNs must have the same rate/weigth");
        end
        
        common_trans_types(index) = gspn1.type_transitions(t_index1);
        common_trans_rates(index) = gspn1.rate_transitions(t_index1);
    end
    
    %Struct to hold information about the arcs that are connected to each common
    %places in both GSPNs
    common_place_arcs = struct();
    
    for p_index = 1:nCommonPlace
        place_name = common_places(p_index);
        place_index1 = p_indices1(p_index);
        place_index2 = p_indices2(p_index);
        common_place_arcs(p_index).place_name = place_name;
        
        %Find indices and then names of all transitions connected to place
        %in common, in both GSPN1 and GSPN2
        target_trans1 = find(gspn1.input_arcs(place_index1, :));
        common_place_arcs(p_index).target_transition1 = translate_to_names(gspn1.transitions, target_trans1);
        target_trans2 = find(gspn2.input_arcs(place_index2, :));
        common_place_arcs(p_index).target_transition2 = translate_to_names(gspn2.transitions, target_trans2);
        source_trans1 = find(gspn1.output_arcs(:, place_index1));
        common_place_arcs(p_index).source_transition1 = translate_to_names(gspn1.transitions, source_trans1);
        source_trans2 = find(gspn2.output_arcs(:, place_index2));
        common_place_arcs(p_index).source_transition2 = translate_to_names(gspn2.transitions, source_trans2);
    end
    
    %Struct to hold information about arcs that are connected to each common
    %transition in both GSPNs
    common_trans_arcs = struct();
    
    for t_index = 1:nCommonTrans
        transition_name = common_trans(t_index);
        transition_index1 = t_indices1(t_index);
        transition_index2 = t_indices2(t_index);
        common_trans_arcs(t_index).transition_name = transition_name;
        
        %Find indices and then names of all places connected to the
        %transition in common, in both GSPN1 and GSPN2
        source_place1 = find(gspn1.input_arcs(:, transition_index1));
        common_trans_arcs(t_index).source_place1 = translate_to_names(gspn1.places, source_place1');
        source_place2 = find(gspn2.input_arcs(:, transition_index2));
        common_trans_arcs(t_index).source_place2 = translate_to_names(gspn2.places, source_place2');
        target_place1 = find(gspn1.output_arcs(transition_index1, :));
        common_trans_arcs(t_index).target_place1 = translate_to_names(gspn1.places, target_place1);
        target_place2 = find(gspn2.output_arcs(transition_index2, :));
        common_trans_arcs(t_index).target_place2 = translate_to_names(gspn2.places, target_place2);
    end
    
    gspn1_copy = copy(gspn1);
    gspn2_copy = copy(gspn2);
    
    %Eliminate common places and transitions from these copy GSPNs
    
    if has_common_places
        gspn1_copy.remove_places(common_places);
        gspn2_copy.remove_places(common_places);
    end
    if has_common_trans
        gspn1_copy.remove_transitions(common_trans);
        gspn2_copy.remove_transitions(common_trans);
    end
    
    %Add all places/transitions/arcs that have no conflict between the 2
    %GSPNS
    
    merged.add_places(gspn1_copy.places, gspn1_copy.initial_marking);
    merged.add_transitions(gspn1_copy.transitions, gspn1_copy.type_transitions, gspn1_copy.rate_transitions);
    merged.add_places(gspn2_copy.places, gspn2_copy.initial_marking);
    merged.add_transitions(gspn2_copy.transitions, gspn2_copy.type_transitions, gspn2_copy.rate_transitions);
    merged.add_arcs(gspn1_copy.arcs.places, gspn1_copy.arcs.transitions, gspn1_copy.arcs.types, gspn1_copy.arcs.weights);
    merged.add_arcs(gspn2_copy.arcs.places, gspn2_copy.arcs.transitions, gspn2_copy.arcs.types, gspn2_copy.arcs.weights);

    %Start adding the common elements, with the corresponding input and
    %output arcs
    arc_places = [string.empty];
    arc_trans = [string.empty];
    arc_types = [string.empty];
    arc_weights = [];
    
    if has_common_places
        merged.add_places(common_places, zeros(1, nCommonPlace));
    end
    if has_common_trans
        merged.add_transitions(common_trans, common_trans_types, common_trans_rates)
    end
    
    %This for loop won't execute if there are no common places
    for p_index = 1:nCommonPlace
        common_place = common_place_arcs(p_index);
        place_name = common_place.place_name;
        nInputArcs  = size(common_place.target_transition1, 2) + size(common_place.target_transition2, 2);
        nOutputArcs = size(common_place.source_transition1, 2) + size(common_place.source_transition2, 2);
        
        inter_trans_list = [];
        %Add all input arcs to the input lists
        arc_places = cat(2, arc_places, repmat(place_name, [1 nInputArcs]));
        inter_trans_list = cat(2, inter_trans_list, common_place.target_transition1);
        inter_trans_list = cat(2, inter_trans_list, common_place.target_transition2);
        arc_trans = cat(2, arc_trans, inter_trans_list);
        arc_types = cat(2, arc_types, repmat("in", [1 nInputArcs]));
        arc_weights = cat(2, arc_weights, ones(1, nInputArcs));
        %Add all output arcs to the input lists
        arc_places = cat(2, arc_places, repmat(place_name, [1 nOutputArcs]));
        inter_trans_list = [];
        inter_trans_list = cat(2, inter_trans_list, common_place.source_transition1);
        inter_trans_list = cat(2, inter_trans_list, common_place.source_transition2);
        arc_trans = cat(2, arc_trans, inter_trans_list);
        arc_types = cat(2, arc_types, repmat("out", [1 nOutputArcs]));
        arc_weights = cat(2, arc_weights, ones(1, nOutputArcs));
        
    end
    
    merged.add_arcs(arc_places, arc_trans, arc_types, arc_weights);
    
    %Reset input lists
    arc_places = [string.empty];
    arc_trans = [string.empty];
    arc_types = [string.empty];
    arc_weights = [];
    %This for loop won't execute if there are no common transitions
    for t_index = 1:nCommonTrans
        common_trans = common_trans_arcs(t_index);
        trans_name = common_trans.transition_name;
        nInputArcs = size(common_trans.source_place1, 2) + size(common_trans.source_place2, 2);
        nOutputArcs = size(common_trans.target_place1, 2) + size(common_trans.target_place2, 2);
        
        inter_place_list = [];
        %Add all input arcs to the input lists
        inter_place_list = cat(2, inter_place_list, common_trans.source_place1);
        inter_place_list = cat(2, inter_place_list, common_trans.source_place2);
        arc_places = cat(2, arc_places, inter_place_list);
        arc_trans = cat(2, arc_trans, repmat(trans_name, [1 nInputArcs]));
        arc_types = cat(2, arc_types, repmat("in", [1 nInputArcs]));
        arc_weights = cat(2, arc_weights, ones(1, nInputArcs));
        inter_place_list = [];
        %Add all output arcs to the input lists
        inter_place_list = cat(2, inter_place_list, common_trans.target_place1);
        inter_place_list = cat(2, inter_place_list, common_trans.target_place2);
        arc_places = cat(2, arc_places, inter_place_list);
        arc_trans = cat(2, arc_trans, repmat(trans_name, [1 nOutputArcs]));
        arc_types = cat(2, arc_types, repmat("out", [1 nOutputArcs]));
        arc_weights = cat(2, arc_weights, ones(1, nOutputArcs));
    end
    
    merged.add_arcs(arc_places, arc_trans, arc_types, arc_weights);
       
end

    

