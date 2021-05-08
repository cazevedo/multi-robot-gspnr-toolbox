function [GSPNRModel, parameters] = test_GSPNRCreationandConversiontoMDP(nLocations)
    parameters = struct();
    % Importing GSPN primitive action models (navigation/mopping/vacuuming)

    PNPRO_path = "meeting_example_notimeouts.PNPRO";

    [nGSPN, models] = ImportfromGreatSPN(PNPRO_path);

    NavigationModel = models.navigation;
    MoppingModel = models.mopping;
    VacuumingModel = models.vacuuming;
 
    % Creating overall GSPNR model using topological map and imported models
    adjacency_matrix = [];
    for r_index = 1:nLocations
        row = zeros(1, nLocations);
        if ~((r_index-1)<1)
            row(r_index-1) = 1;
        end
        if ~((r_index+1)>nLocations)
            row(r_index+1) = 1;
        end
        adjacency_matrix(r_index,:) = row;
    end
    node_array = {};
    for l_index = 1:nLocations
        array = {'L', int2str(l_index)};
        node_name = strjoin(array,'');
        node_array = [node_array, node_name];
    end
        
    topological_map = digraph(adjacency_matrix, node_array, 'omitselfloops');
    actions_available = struct();
    plot(topological_map)
    for l_index = 1:nLocations
        l_name = node_array{l_index};
        actions_available.(l_name) = ["mopping", "vacuuming"];
    end
    robot_marking = struct();
    robot_marking.L1 = 1;
    robot_marking.(node_array{nLocations}) = 1;
    tic
    GSPNRModel = GSPNRCreationfromTopMap(topological_map, actions_available, models, robot_marking);
    parameters.GSPNR_creation_time = toc;
    parameters.nPlaces = size(GSPNRModel.places,2);
    parameters.nTransitions = size(GSPNRModel.transitions,2);
    parameters.nEdges = size(topological_map.Edges,1);
    tic
    [emb_MDP, covered_marking_list, covered_state_list, covered_state_type] = GSPNRModel.toMDP();
    parameters.MDP_creation_time = toc;
    parameters.nStates = emb_MDP.nStates;
%     parameters.instant = datestr(now);
end

