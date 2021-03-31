function MergeGSPNR(gspn1,gspn2)
%MERGEGSPNR Summary of this function goes here
%   Detailed explanation goes here

    merged = GSPNR();

    places1 = gspn1.places;
    places2 = gspn2.places;
    
    trans1 = gspn1.transitions;
    trans2 = gspn2.transitions;
    
    [common_places, p_indices1, p_indices2] = intersect(places1, places2);
    [common_trans,  t_indices1, t_indices2] = intersect(trans1, trans2);
    
    %Perform sanity checks on transitions - Check if transitions common to
    %both GSPNRs have same type and same weight/rate;
    nCommonPlace = size(common_places, 2);
    nCommonTrans = size(common_trans, 2);
    
    for index = 1:nCommonTrans
        trans_name = common_trans(index);
        t_index1 = t_indices1(index);
        t_index2 = t_indices2(index);
        
        if gspn1.type_transitions(t_index1) ~= gspn2.type_transitions(t_index2)
            error("Common transitions between the two GSPNs must be the same type");
        elseif gspn1.rate_transitions(t_index1) ~= gspn2.rate_transitions(t_index2)
            error("Common transitions between the two GSPNs must have the same rate/weigth");
        end
    end
    
    %Struct to hold information about the arcs that are connected to common
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
    
    %Struct to hold information about arcs that are connected to common
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
    
    gspn1_copy.remove_places(common_places);
    gspn1_copy.remove_transitions(common_trans);
    gspn2_copy.remove_places(common_places);
    gspn2_copy.remove_transitions(common_trans);
    
    %Add all places/transitions/arcs that have no conflict between the 2
    %GSPNS
    
    merged.add_places(gspn1_copy.places, gspn1_copy.initial_marking);
    merged.add_transitions(gspn1_copy.transitions, gspn1_copy.type_transitions, gspn1_copy.rate_transitions);
    merged.add_places(gspn2_copy.places, gspn2_copy.initial_marking);
    merged.add_transitions(gspn2_copy.transitions, gspn2_copy.type_transitions, gspn2_copy.rate_transitions);
    merged.add_arcs(gspn1_copy.arcs.places, gspn1_copy.arcs.transitions, gspn1_copy.arcs.types, gspn1_copy.arcs.weights);
    merged.add_arcs(gspn2_copy.arcs.places, gspn2_copy.arcs.transitions, gspn2_copy.arcs.types, gspn2_copy.arcs.weights);


end

    

