function exec = GSPNRCreationfromTopMap(top_map,action_dict, models)
%GSPNRCREATIONFROMTOPMAP Summary of this function goes here
%   %obj = ExecutableGSPNR(top_map, action_dict, models)
            %exec = GSPNR()
            %foreach NODE in top_map
            %   action_list = action_dict.(NODE)
            %   decision_place = 'dec_'+str(NODE)
            %   foreach ACTION in action_list
            %       gspn_instance = copy(models.(ACTION))
            %       gspn_instance.format(decision_place)
            %       exec = MergeGSPNR(exec, gspn_instance)
            %foreach EDGE in top_map.NODE
            %   gspn_instance = copy(models.navigation)
            %   gspn_instance.format(decision_place, top_map.edge(2))
            %   exec = MergeGSPNR(exec, gspn_instance)
            
    %Check that all actions in action dict have a corresponding model in
    %the models input + check that there is a navigation action
    if ~isfield(models, "navigation")
        error("There is a missing model for the NAVIGATION action");
    end
    nodes_with_actions = string(fieldnames(action_dict));
    nNodesWithActions = size(nodes_with_actions, 1);
    for n_index = 1:nNodesWithActions
        node = nodes_with_actions(n_index);
        action_list = action_dict.(node);
        nActions = size(action_list, 2);
        for a_index = 1:nActions
            action = action_list(a_index);
            if ~isfield(models, action)
                error_string = "There is a missing model for action - "+action;
                error(error_string);
            end
        end
    end
    %Creating empty GSPNR object to start building on        
    exec = GSPNR();
    %Iterating through list of nodes and adding the action GSPNR models +
    %decision places
    nodes = string(table2array(top_map.Nodes));
    nNodes = size(nodes, 1);
    for n_index = 1:nNodes
        node_name = nodes(n_index);
        action_list = action_dict.(node_name);
        nActions = size(action_list, 2);
        for a_index = 1:nActions
            action_name = action_list(a_index);
            action_gspn = copy(models.(action_name));
            action_gspn.format([node_name]);
            exec = MergeGSPNR(exec, action_gspn);
        end
    end
    edges = string(top_map.Edges.EndNodes);
    nEdges = size(edges, 1);
    for e_index = 1:nEdges
        source = edges(e_index, 1);
        target = edges(e_index, 2);
        navigation_gspn = copy(models.navigation);
        navigation_gspn.format([source, target]);
        exec = MergeGSPNR(exec, navigation_gspn);
    end
end

