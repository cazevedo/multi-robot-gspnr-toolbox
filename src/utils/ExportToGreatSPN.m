function ExportToGreatSPN(test,savepath, GreatSPN_size)
%EXPORTTOGREATSPN Summary of this function goes here
%   Detailed explanation goes here

    project = struct();

    project.nameAttribute = "test";
    project.versionAttribute = "121";

    project.gspn.nameAttribute = "test";
    project.gspn.show_color_cmdAttribute = "false";
    project.gspn.show_fluid_cmdAttribute = "false";

    nPlaces = size(test.places, 2);
    for p_index = 1:nPlaces
        place_name = test.places(p_index);
        ntokens = test.initial_marking(p_index);
        pos_x = randi([0 GreatSPN_size(1)]);
        pos_y = randi([0 GreatSPN_size(2)]);
        project.gspn.nodes.place(p_index) = format_place_template(place_name, [pos_x pos_y], ntokens);
    end
    nTransitions = size(test.transitions,2);
    for t_index = 1:nTransitions
        name = test.transitions(t_index);
        rate = test.rate_transitions(t_index);
        if test.type_transitions(t_index) == "imm"
            timed = false;
        else
            timed = true;
        end
        pos_x = randi([0 GreatSPN_size(1)]);
        pos_y = randi([0 GreatSPN_size(1)]);
       project.gspn.nodes.transition(t_index) = format_transition_template(name, [pos_x pos_y], timed, rate);
    end
    %Save input arcs
    nArcs = 0;
    for p_index = 1:nPlaces
        for t_index = 1:nTransitions
            source = test.places(p_index);
            target = test.transitions(t_index);
            weight = test.input_arcs(p_index,t_index);
            if weight == 0
                continue;
            end
            nArcs = nArcs + 1;
            project.gspn.edges.arc(nArcs) = format_arc_template(source, target, weight, true);
        end
    end
    %Save output arcs
    for t_index = 1:nTransitions
        for p_index = 1:nPlaces
            source = test.transitions(t_index);
            target = test.places(p_index);
            weight = test.output_arcs(t_index,p_index);
            if weight == 0
                continue;
            end
            nArcs = nArcs + 1;
            project.gspn.edges.arc(nArcs) = format_arc_template(source, target, weight, false);
        end
    end
    writestruct(project, savepath, "StructNodeName","project",'FileType','xml')
end

function place_struct = format_place_template(name, pos, ntokens)
    if ntokens ~= 0
        place_struct.markingAttribute = ntokens;
    else
        place_struct.markingAttribute = missing;
    end
    place_struct.nameAttribute = name;
    place_struct.xAttribute = pos(1);
    place_struct.yAttribute = pos(2);
end

function trans_struct = format_transition_template(name, pos, timed, rate)
    if timed == false
        trans_struct.delayAttribute = missing;
        trans_struct.weightAttribute = rate;
        trans_struct.typeAttribute = "IMM";
    else
        trans_struct.delayAttribute = rate;
        trans_struct.weightAttribute = missing;
        trans_struct.typeAttribute = "EXP";
    end
    trans_struct.nameAttribute = name;
    trans_struct.xAttribute = pos(1);
    trans_struct.yAttribute = pos(2);
end

function arc_struct = format_arc_template(source, target, weight, input)
    arc_struct.headAttribute = target;
    arc_struct.tailAttribute = source;
    arc_struct.multAttribute = weight;
    if input == true
        arc_struct.kindAttribute = "INPUT";
    else
        arc_struct.kindAttribute = "OUTPUT";
    end
end

