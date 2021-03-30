function [nGSPN, gspn_array] = ImportfromGreatSPN(PNPRO_path)
%IMPORTFROMGREATSPN Summary of this function goes here
%   Detailed explanation goes here
    
    gspn_array = {};

    struct = readstruct(PNPRO_path, 'Filetype', 'xml');
    
    nGSPN = size(struct.gspn, 2);
    
    for g_index = 1:nGSPN
        
        gspn = GSPNR();
        gspn_struct = struct.gspn(g_index);
        
        nPlaces = size(gspn_struct.nodes.place,2);
        places = [string.empty];
        tokens = [];

        nTrans = size(gspn_struct.nodes.transition,2);
        trans_names = [string.empty];
        trans_types = [string.empty];
        trans_rates = [];
        
        nArcs = size(gspn_struct.edges.arc,2);
        arc_places = [string.empty];
        arc_trans = [string.empty];
        arc_type = [string.empty];
        arc_weight = [];

        for p_index = 1:nPlaces
            place_name = gspn_struct.nodes.place(p_index).nameAttribute;
            if ismissing(gspn_struct.nodes.place(p_index).markingAttribute)
                ntokens = 0;
            else
                ntokens = gspn_struct.nodes.place(p_index).markingAttribute;
            end
            places = cat(2, places, place_name);
            tokens = cat(2, tokens, ntokens);
        end
        gspn.add_places(places,tokens);
        for t_index = 1:nTrans
            trans_name = gspn_struct.nodes.transition(t_index).nameAttribute;
            trans_type = gspn_struct.nodes.transition(t_index).typeAttribute;
            if trans_type == "EXP"
                trans_type = "exp";
                trans_rate = gspn_struct.nodes.transition(t_index).delayAttribute;
            else
                trans_type = "imm";
                if ismissing(gspn_struct.nodes.transition(t_index).weightAttribute);
                    trans_rate = 0;
                else
                    trans_rate = gspn_struct.nodes.transition(t_index).weightAttribute;
                end
            end
            trans_names = cat(2, trans_names, trans_name);
            trans_types = cat(2, trans_types, trans_type);
            trans_rates = cat(2, trans_rates, trans_rate);
        end
        gspn.add_transitions(trans_names, trans_types, trans_rates);
        for a_index = 1:nArcs
            if gspn_struct.edges.arc(a_index).kindAttribute == "INPUT"
                type = "in";
                place = gspn_struct.edges.arc(a_index).tailAttribute;
                trans = gspn_struct.edges.arc(a_index).headAttribute;
                if ismissing(gspn_struct.edges.arc(a_index).multAttribute)
                    weight = 1;
                else
                    weight = gspn_struct.edges.arc(a_index).multAttribute
                end
            else
                type = "out";
                place = gspn_struct.edges.arc(a_index).headAttribute;
                trans = gspn_struct.edges.arc(a_index).tailAttribute;
                if ismissing(gspn_struct.edges.arc(a_index).multAttribute)
                    weight = 1;
                else
                    weight = gspn_struct.edges.arc(a_index).multAttribute
                end
            end
            arc_places = cat(2, arc_places, place);
            arc_trans  = cat(2, arc_trans, trans);
            arc_type   = cat(2, arc_type, type);
            arc_weight = cat(2, arc_weight, weight);
        end
        gspn.add_arcs(arc_places, arc_trans, arc_type, arc_weight);
        gspn_array{g_index} = gspn;
    end
end
