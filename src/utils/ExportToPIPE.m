function ExportToPIPE(GSPNR, save_path, PIPE_size)
    
    pnml = struct();

    pnml.net.idAttribute = "Net-One";
    pnml.net.typeAttribute = "P/T net";

    load("token_default.mat", "token_template")
    % load("transition_template.mat","transition_template")
    % load("arc_template.mat","arc_template")

    pnml.net.token = token_template;

    %Save places
    nPlaces = size(GSPNR.places, 2);
    for p_index = 1:nPlaces
        place_name = GSPNR.places(p_index);
        ntokens = GSPNR.initial_marking(p_index);
        pos_x = randi([0 PIPE_size(1)]);
        pos_y = randi([0 PIPE_size(2)]);
        pnml.net.place(p_index) = format_place_template(place_name, [pos_x pos_y], ntokens);
    end
    %Save transitions
    nTransitions = size(GSPNR.transitions,2);
    for t_index = 1:nTransitions
        name = GSPNR.transitions(t_index);
        rate = GSPNR.rate_transitions(t_index);
        if GSPNR.type_transitions(t_index) == "imm"
            timed = false;
        else
            timed = true;
        end
        pos_x = randi([0 PIPE_size(1)]);
        pos_y = randi([0 PIPE_size(2)]);
        pnml.net.transition(t_index) = format_transition_template(name, [pos_x pos_y], timed, rate);
    end
    %Save input arcs
    nArcs = 0;
    for p_index = 1:nPlaces
        for t_index = 1:nTransitions
            source = GSPNR.places(p_index);
            target = GSPNR.transitions(t_index);
            weight = GSPNR.input_arcs(p_index,t_index);
            if weight == 0
                continue;
            end
            nArcs = nArcs + 1;
            pnml.net.arc(nArcs) = format_arc_template(source, target, weight);
        end
    end
    %Save output arcs
    for t_index = 1:nTransitions
        for p_index = 1:nPlaces
            source = GSPNR.transitions(t_index);
            target = GSPNR.places(p_index);
            weight = GSPNR.output_arcs(t_index,p_index);
            if weight == 0
                continue;
            end
            nArcs = nArcs + 1;
            pnml.net.arc(nArcs) = format_arc_template(source, target, weight);
        end
    end

    writestruct(pnml, save_path, "StructNodeName","pnml")
end

function place_struct = format_place_template(name, pos, ntokens)
    load("place_template.mat","place_template");
    place_template.idAttribute = name;
    place_template.name.value = name;
    place_template.graphics.position.xAttribute = pos(1);
    place_template.graphics.position.yAttribute = pos(2);
    place_template.initialMarking.value = "Default,"+string(ntokens);
    place_struct = place_template;
end

function trans_struct = format_transition_template(name, pos, timed, rate)
    load("transition_template.mat", "transition_template");
    transition_template.idAttribute = name;
    transition_template.graphics.position.xAttribute = pos(1);
    transition_template.graphics.position.yAttribute = pos(2);
    transition_template.name.value = name;
    transition_template.rate.value = rate;
    if timed == false
        transition_template.timed.value = "false";
    else
        transition_template.timed.value = "true";
    end
    trans_struct = transition_template;
end

function arc_struct = format_arc_template(source, target, weight)
    load("arc_template.mat","arc_template");
    arc_template.idAttribute = source+" to "+target;
    arc_template.sourceAttribute = source;
    arc_template.targetAttribute = target;
    arc_template.inscription.value = "Default,"+string(weight);
    arc_struct = arc_template;
end

