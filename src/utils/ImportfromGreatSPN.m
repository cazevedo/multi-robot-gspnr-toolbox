function [nGSPN, gspn_struct_array] = ImportfromGreatSPN(PNPRO_path)
%Imports GSPNR object from PNPRO file exported from GreatSPN software

    
    struct = readstruct(PNPRO_path, 'Filetype', 'xml');
    
    nGSPN = size(struct.gspn, 2);
    
    for g_index = 1:nGSPN
        
        gspn = GSPNR();
        gspn_struct = struct.gspn(g_index);
        
        if strcmp(gspn_struct.nodes, "")
            nPlaces = 0;
            nTrans = 0;
        else
            if isfield(gspn_struct.nodes, "place")
                nPlaces = size(gspn_struct.nodes.place,2);
            else
                nPlaces = 0;
            end
            if isfield(gspn_struct.nodes, "transition")
                nTrans = size(gspn_struct.nodes.transition,2);
            else
                nTrans = 0;
            end
        end
        
        places = [string.empty];
        tokens = [];
        trans_names = [string.empty];
        trans_types = [string.empty];
        trans_rates = [];
        
        if strcmp(gspn_struct.edges, "")
            nArcs = 0;
        else
            nArcs = size(gspn_struct.edges.arc,2);
        end
        arc_places = [string.empty];
        arc_trans = [string.empty];
        arc_type = [string.empty];
        arc_weight = [];
        
        no_tokens = false;
        for p_index = 1:nPlaces
            fields = string(fieldnames(gspn_struct.nodes.place(p_index)));
            if isempty(find(fields == 'markingAttribute'))
                no_tokens = true;
            end
            place_name = gspn_struct.nodes.place(p_index).nameAttribute;
            if no_tokens == true
                ntokens = 0;
            else
                if ismissing(gspn_struct.nodes.place(p_index).markingAttribute)
                    ntokens = 0;
                else
                    ntokens = gspn_struct.nodes.place(p_index).markingAttribute;
                end
            end
            places = cat(2, places, place_name);
            tokens = cat(2, tokens, ntokens);
        end
        gspn.add_places(places,tokens);
        no_delays = false;
        no_weights = false;
        for t_index = 1:nTrans
            fields = string(fieldnames(gspn_struct.nodes.transition(t_index)));
            if isempty(find(fields == 'delayAttribute'))
                no_delays = true;
            end
            if isempty(find(fields == 'weightAttribute'))
                no_weights = true;
            end
            trans_name = gspn_struct.nodes.transition(t_index).nameAttribute;
            trans_type = gspn_struct.nodes.transition(t_index).typeAttribute;
            if trans_type == "EXP"
                trans_type = "exp";
                if no_delays == true
                    trans_rate = 1;
                else
                    trans_rate = gspn_struct.nodes.transition(t_index).delayAttribute;
                end
            else
                trans_type = "imm";
                if no_weights == true
                    trans_rate = 0;
                else
                    if ismissing(gspn_struct.nodes.transition(t_index).weightAttribute);
                        trans_rate = 0;
                    else
                        trans_rate = gspn_struct.nodes.transition(t_index).weightAttribute;
                    end
                end
            end
            trans_names = cat(2, trans_names, trans_name);
            trans_types = cat(2, trans_types, trans_type);
            trans_rates = cat(2, trans_rates, trans_rate);
        end
        gspn.add_transitions(trans_names, trans_types, trans_rates);
        no_arc_weights = false;
        for a_index = 1:nArcs
            fields = string(fieldnames(gspn_struct.edges.arc(a_index)));
            if isempty(find(fields == 'multAttribute'))
                no_arc_weights = true;
            end
            if gspn_struct.edges.arc(a_index).kindAttribute == "INPUT"
                type = "in";
                place = gspn_struct.edges.arc(a_index).tailAttribute;
                trans = gspn_struct.edges.arc(a_index).headAttribute;
                if no_arc_weights == true
                    weight = 1;
                else
                    if ismissing(gspn_struct.edges.arc(a_index).multAttribute)
                        weight = 1;
                    else
                        weight = gspn_struct.edges.arc(a_index).multAttribute
                    end
                end
            else
                type = "out";
                place = gspn_struct.edges.arc(a_index).headAttribute;
                trans = gspn_struct.edges.arc(a_index).tailAttribute;
                if no_arc_weights == true
                    weight = 1;
                else
                    if ismissing(gspn_struct.edges.arc(a_index).multAttribute)
                        weight = 1;
                    else
                        weight = gspn_struct.edges.arc(a_index).multAttribute
                    end
                end
            end
            arc_places = cat(2, arc_places, place);
            arc_trans  = cat(2, arc_trans, trans);
            arc_type   = cat(2, arc_type, type);
            arc_weight = cat(2, arc_weight, weight);
        end
        gspn.add_arcs(arc_places, arc_trans, arc_type, arc_weight);
        gspn_struct_array.(gspn_struct.nameAttribute) = gspn;
    end
end

