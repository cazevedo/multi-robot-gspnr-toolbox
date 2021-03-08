classdef MarkovDecisionProblem < handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        states = [string.empty]; %list of strings, each string is the name of the state
        nStates = 0;% total number of states
        actions = [string.empty]; %list of strings, each string is an immediate action
        nActions = 0;%total number of immediate actions
        exp_actions =[string.empty];%list of strings, each string is an exponential action
        nEXPActions = 0;%total number of exponential actions
        transition_matrix = {};%cell array where the first column holds the indices of the transition, and the second column holds the rate of transition
        nTransitions = 0;%number of elements in transition matrix, eq. to number of nonzero elements in conventional matrix
        exponential_transition_matrix = {};
        nEXPTransitions = 0;%number of elements in exponential transition matrix
        initial_state = [];
        nRewards = 0;
        reward_matrix = [];
        eta = 0;
    end
    
    methods
        function state_index = add_state(MDP, state_name)
            if isempty(find(MDP.states == state_name))   
                MDP.states = cat(2, MDP.states, state_name);
                MDP.nStates = MDP.nStates+1;
                state_index = length(MDP.states);
            else
                disp("WARNING: Tried to add state that already exists")
                state_index = find(MDP.states == state_name);
            end
        end
        function state_index = find_state(MDP, state_name)
            index = find(MDP.states == state_name);
            if isempty(index)
                disp("State does not exist")
                state_index = 0;
            else
                state_index = index;
            end
        end
        function action_index = add_action(MDP, action_name, type)
            if (type == "imm")
                if isempty(find(MDP.actions == action_name))
                    MDP.actions = cat(2, MDP.actions, action_name);
                    MDP.nActions = MDP.nActions+1;
                    action_index = length(MDP.actions);

                else
                    disp("WARNING: Tried to add action that already exists")
                    action_index = find(MDP.actions == action_name);
                end
            elseif (type == "exp")
                if isempty(find(MDP.exp_actions == action_name))
                    MDP.exp_actions = cat(2, MDP.exp_actions, action_name);
                    action_index = length(MDP.exp_actions);

                else
                    disp("WARNING: Tried to add action that already exists")
                    action_index = find(MDP.exp_actions == action_name);
                end
            else
                error("Action added must be either immediate or exponential")
            end
        end
        
        function action_index = find_action(MDP, action_name, type)
            if type == "imm"
                index = find(MDP.actions == action_name);
            elseif type == "exp"
                index = find(MDP.exp_actions == action_name);
            else
                error("Action type must be either 'imm' or 'exp'");
            end
            if isempty(index)
                index = 0;
                action_index = index;
                disp("Action does not exist")
            else
                action_index = index;
            end
        end
        
        function set_transition(MDP, source_state, action, end_state, prob, type)
            source_state_index = MDP.find_state(source_state);
            action_index = MDP.find_action(action, type);
            end_state_index = MDP.find_state(end_state);
            
            if ~(source_state_index && action_index && end_state_index)
                error("Tried to add transition from nonexistent element");
            else
                indices = [source_state_index, action_index, end_state_index];
                if type == "imm"
                    %Add to deterministic transition matrix
                    MDP.nTransitions = MDP.nTransitions+1;
                    MDP.transition_matrix{MDP.nTransitions,1} = indices;
                    MDP.transition_matrix{MDP.nTransitions,2} = prob;
                elseif type == "exp"
                    %Add to exponential transition matrix
                    MDP.nEXPTransitions = MDP.nEXPTransitions+1;
                    MDP.exponential_transition_matrix{MDP.nEXPTransitions,1} = indices;
                    MDP.exponential_transition_matrix{MDP.nEXPTransitions,2} = prob;
                else
                    error("Type of transition must be either 'imm' or 'exp'");
                end
            end
        end
        
        function [full_transition, full_rewards] = get_full_matrices(MDP)
            full_transition = zeros(MDP.nStates, MDP.nActions, MDP.nStates);
            full_rewards = zeros(MDP.nStates, MDP.nActions);
            
            for row_index = 1:MDP.nTransitions
                indices = MDP.transition_matrix{row_index, 1};
                prob = MDP.transition_matrix{row_index, 2};
                full_transition(indices(1),indices(2),indices(3)) = prob;
            end
            for row_index = 1:MDP.nRewards
                indices = MDP.reward_matrix{row_index,1};
                reward = MDP.reward_matrix{row_index, 2};
                full_rewards(indices(1),indices(2)) = reward;
            end
        end
        
        function set_reward(MDP, state, action, reward)
            state_index = MDP.find_state(state);
            action_index = MDP.find_action(action, "imm");
            
            if ~(state_index && action_index)
                error("Tried to add reward to nonexistent element");
            else
                indices = [state_index, action_index];
                MDP.nRewards = MDP.nRewards + 1;
                MDP.reward_matrix{MDP.nRewards, 1} = indices;
                MDP.reward_matrix{MDP.nRewards, 2} = reward;
            end
        end
                
        
        function consolidation_uniformization(MDP, state_types)
           %Calculation of eta, uniformization constant
           exp_action_matrix = zeros(MDP.nStates, MDP.nStates);
           total_trans_freq = zeros(MDP.nStates, MDP.nStates);
           exit_rates = [];
           for row_index = 1:MDP.nEXPTransitions
               indices = MDP.exponential_transition_matrix{row_index, 1};
               rate = MDP.exponential_transition_matrix{row_index, 2};
               total_trans_freq(indices(1),indices(3)) = total_trans_freq(indices(1),indices(3)) + rate;
           end
           exit_rates = sum(total_trans_freq, 2);
           eta = max(exit_rates) + 1;
           MDP.eta = eta;
           %Uniformization and Normalization for Exponential Transitions
           for source_state = 1:MDP.nStates
               for target_state = 1:MDP.nStates
                   if source_state == target_state
                       %In diagonal, transition that begins and ends in
                       %same state/marking
                       state_type = state_types(source_state);
                       if state_type == "TAN"
                        exp_action_matrix(source_state, target_state) = 1 - (exit_rates(source_state)-total_trans_freq(source_state,target_state))*(1/eta);
                       end
                   else
                       %Nondiagonal, normal expression
                       exp_action_matrix(source_state, target_state) = total_trans_freq(source_state,target_state)/eta;
                   end
               end
           end
           exponential_action_index = MDP.add_action("EXP","imm");
           
           [source_state_index, end_state_index, val] = find(exp_action_matrix);
           nonZeroElements = length(val);
           for element = 1:nonZeroElements
               source_state_name = MDP.states(source_state_index(element));
               end_state_name = MDP.states(end_state_index(element));
               MDP.set_transition(source_state_name, "EXP", end_state_name, val(element), "imm");
           end
               
           MDP.exponential_transition_matrix = [];
           MDP.nEXPActions = 0;
           MDP.exp_actions = [string.empty];
        end
        function [validity, cumulative_prob] = check_validity(MDP)
            cumulative_prob = zeros(MDP.nStates, MDP.nActions);
            for row = 1:MDP.nTransitions
                indices = MDP.transition_matrix{row,1};
                prob = MDP.transition_matrix{row,2};
                source_state_index = indices(1);
                action_index = indices(2);
                cumulative_prob(source_state_index, action_index) = cumulative_prob(source_state_index,action_index)+prob;                
            end
            [source_index, action_index, cum_prob] = find(cumulative_prob);
            if (any(cum_prob~=1))
                validity = false;
            else
                validity = true;
            end      
        end
        function [actions_enabled] = actions_enabled(MDP, state)
            actions_enabled = [string.empty];
            [validity, cumulative_prob] = MDP.check_validity();
            if ~validity
                error("MDP does not have actions with correct transition probabilities");
            end
            state_index = MDP.find_state(state);
            row = cumulative_prob(state_index, :);
            action_indices = find(row);
            for index = 1:length(action_indices)
                action_name = MDP.actions(action_indices(index));
                actions_enabled = cat(1, actions_enabled, action_name);
            end
        end
            
    end
    
end


