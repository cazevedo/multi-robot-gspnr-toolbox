classdef ExecutableGSPNR < GSPNR
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        place_actions = struct();
    end
    
    methods
        function obj = ExecutableGSPNR(GSPNR, action_map)
            %obj = ExecutableGSPNR(top_map, action_dict, models)
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
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

