classdef MarkovDecisionProblem < handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        states = [string.empty];
        nStates = 0;
        actions = [string.empty];
        nActions = 0;
        exp_actions =[string.empty];
        nEXPActions = 0;
        transition_matrix = [];
        exponential_transition_matrix = [];
        initial_state = [];
        reward_matrix = [];
    end
    
    methods
        function state_index = add_state(MDP, state_name)
            if isempty(find(MDP.states == state_name))   
                MDP.states = cat(2, MDP.states, state_name);
                MDP.nStates = MDP.nStates+1;
                MDP.transition_matrix = padarray(MDP.transition_matrix, [1 0 1],0,'post');
                MDP.transition_matrix = MDP.transition_matrix(:,:,1:MDP.nStates);
                
                MDP.exponential_transition_matrix = padarray(MDP.exponential_transition_matrix, [1 0 1],0,'post');
                MDP.exponential_transition_matrix = MDP.exponential_transition_matrix(:,:,1:MDP.nStates);
                state_index = length(MDP.states);
            else
                disp("WARNING: Tried to add state that already exists")
                state_index = find(MDP.states == state_name);
            end
        end
        function action_index = add_action(MDP, action_name, type)
            if (type == "imm")
                if isempty(find(MDP.actions == action_name))
                    MDP.actions = cat(2, MDP.actions, action_name);
                    MDP.nActions = MDP.nActions+1;
                    MDP.transition_matrix = padarray(MDP.transition_matrix, [0 1 0],0,'post');
                    action_index = length(MDP.actions);

                else
                    disp("WARNING: Tried to add action that already exists")
                    action_index = find(MDP.actions == action_name);
                end
            elseif (type == "exp")
                if isempty(find(MDP.exp_actions == action_name))
                    MDP.exp_actions = cat(2, MDP.exp_actions, action_name);
                    MDP.nEXPActions = MDP.nEXPActions+1;
                    MDP.exponential_transition_matrix = padarray(MDP.exponential_transition_matrix, [0 1 0],0,'post');
                    action_index = length(MDP.exp_actions);

                else
                    disp("WARNING: Tried to add action that already exists")
                    action_index = find(MDP.exp_actions == action_name);
                end
            else
                error("Action added must be either immediate or exponential")
            end
        end
        function normalization(MDP)
            
        end
                
                
    end
    
end


