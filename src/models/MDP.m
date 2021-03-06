classdef MDP < handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        states = [string.empty];                    %list of strings, each string is the name of the state
        nStates = 0;                                % total number of states
        actions = [string.empty];                   %list of strings, each string is an immediate action
        nActions = 0;                               %total number of immediate actions
        
        transition_matrix = {[], []};               %cell array where the first column holds the indices of the transition, and the second column holds the rate of transition
        nTransitions = 0;                           %number of elements in transition matrix, eq. to number of nonzero elements in conventional matrix
        
        exp_actions =[string.empty];            %list of strings, each string is an exponential action
        nEXPActions = 0;                        %total number of exponential actions
        exponential_transition_matrix = {[], []};
        nEXPTransitions = 0;                    %number of elements in exponential transition matrix
        eta = 0;
        consolidated = false;
        
        nRewards = 0;
        reward_matrix = {[], []};
        
        cumulative_prob = [];
        valid = false;
        enabled_actions = {};
        prepared = false;
        
        initial_state = [];
    end
    
    methods
        function state_index = add_state(MDP, state_name)
            if isempty(find(MDP.states == state_name))   
                MDP.states = cat(2, MDP.states, state_name);
                MDP.nStates = MDP.nStates+1;
                state_index = length(MDP.states);
            else
%                 disp("WARNING: Tried to add state that already exists")
                state_index = find(MDP.states == state_name);
            end
        end
        function state_index = find_state(MDP, state_name)
            index = find(MDP.states == state_name);
            if isempty(index)
%                 disp("State does not exist")
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
%                     disp("WARNING: Tried to add action that already exists")
                    action_index = find(MDP.actions == action_name);
                end
            elseif (type == "exp")
                if isempty(find(MDP.exp_actions == action_name))
                    MDP.exp_actions = cat(2, MDP.exp_actions, action_name);
                    action_index = length(MDP.exp_actions);

                else
%                     disp("WARNING: Tried to add action that already exists")
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
%                     MDP.transition_matrix{MDP.nTransitions,1} = indices;
%                     MDP.transition_matrix{MDP.nTransitions,2} = prob;
                    MDP.transition_matrix{1} = cat(1,MDP.transition_matrix{1}, indices);
                    MDP.transition_matrix{2} = cat(1,MDP.transition_matrix{2}, prob);
                elseif type == "exp"
                    %Add to exponential transition matrix
                    MDP.nEXPTransitions = MDP.nEXPTransitions+1;
%                     MDP.exponential_transition_matrix{MDP.nEXPTransitions,1} = indices;
%                     MDP.exponential_transition_matrix{MDP.nEXPTransitions,2} = prob;
                    MDP.exponential_transition_matrix{1} = cat(1,MDP.exponential_transition_matrix{1}, indices);
                    MDP.exponential_transition_matrix{2} = cat(1,MDP.exponential_transition_matrix{2}, prob);
                else
                    error("Type of transition must be either 'imm' or 'exp'");
                end
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
%                 MDP.reward_matrix{MDP.nRewards, 1} = indices;
%                 MDP.reward_matrix{MDP.nRewards, 2} = reward;
                MDP.reward_matrix{1} = cat(1,MDP.reward_matrix{1}, indices);
                MDP.reward_matrix{2} = cat(1,MDP.reward_matrix{2}, reward);
            end
        end
        
        function full_transition = get_full_transition_matrix(MDP)
            if MDP.valid ~= true
                error("Before calculating full transition matrix, the MDP must be a valid one")
            end
            full_transition = zeros(MDP.nStates, MDP.nActions, MDP.nStates);
            for row_index = 1:MDP.nTransitions
                indices = MDP.transition_matrix{1}(row_index, :);
                prob = MDP.transition_matrix{2}(row_index);
                full_transition(indices(1),indices(2),indices(3)) = prob;
            end
        end
        
        function full_rewards = get_full_reward_matrix(MDP)
            if MDP.valid ~= true
                error("Before calculating full transition matrix, the MDP must be a valid one")
            end
            full_rewards = zeros(MDP.nStates, MDP.nActions);
            for row_index = 1:MDP.nRewards
                indices = MDP.reward_matrix{1}(row_index, :);
                reward = MDP.reward_matrix{2}(row_index);
                full_rewards(indices(1),indices(2)) = reward;
            end
        end
        
        function consolidation_uniformization(MDP, state_types)
           %Calculation of eta, uniformization constant
%            exp_action_matrix = sparse(MDP.nStates, MDP.nStates);
%            total_trans_freq = sparse(MDP.nStates, MDP.nStates);
           
           %Create only exp_action_matrix with index and value list;
           %exp_action_matrix = sparse(MDP.nStates, MDP.nStates);
           
           total_trans_freq = spalloc(MDP.nStates, MDP.nStates, MDP.nEXPTransitions);
           
           exit_rates = [];
           for row_index = 1:MDP.nEXPTransitions
               indices = MDP.exponential_transition_matrix{1}(row_index, :);
               rate = MDP.exponential_transition_matrix{2}(row_index);
               total_trans_freq(indices(1),indices(3)) = total_trans_freq(indices(1),indices(3)) + rate;
           end
           exit_rates = sum(total_trans_freq, 2);
           eta = max(exit_rates) + 1;
           MDP.eta = eta;
           
           exp_action_matrix_indices_i = [];
           exp_action_matrix_indices_j = [];
           exp_action_matrix_value = [];
           %Do diagonals because expression is different
           for state_index = 1:MDP.nStates
               if state_types(state_index) == "TAN"
                   exp_action_matrix_indices_i = cat(1, exp_action_matrix_indices_i, state_index);
                   exp_action_matrix_indices_j = cat(1, exp_action_matrix_indices_j, state_index);
                   value = 1 - (exit_rates(state_index)-total_trans_freq(state_index,state_index))*(1/eta);
                   exp_action_matrix_value = cat(1, exp_action_matrix_value, value);
               end
           end
           [source_state_indices, end_state_indices, rates] = find(total_trans_freq);
           nNonZero = size(source_state_indices, 1);
           for element_index = 1:nNonZero
               exp_action_matrix_indices_i = cat(1, exp_action_matrix_indices_i, source_state_indices(element_index));
               exp_action_matrix_indices_j = cat(1, exp_action_matrix_indices_j, end_state_indices(element_index));
               value = (rates(element_index))/eta;
               exp_action_matrix_value = cat(1, exp_action_matrix_value, value);
           end
           
           exp_action_matrix = sparse( exp_action_matrix_indices_i, exp_action_matrix_indices_j, exp_action_matrix_value);
           
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
           MDP.consolidated = true;
        end
        function check_validity(MDP)
            %Function that simultaneously checks the validity of the
            %transition matrix (ensures that for each state/action pair,
            %the probability sums to either 0 or 1) and sets the cumulative
            %probability property of the MDP.
            MDP.cumulative_prob = sparse(MDP.nStates, MDP.nActions);
            for row = 1:MDP.nTransitions
                indices = MDP.transition_matrix{1}(row, :);
                prob = MDP.transition_matrix{2}(row);
                source_state_index = indices(1);
                action_index = indices(2);
                MDP.cumulative_prob(source_state_index, action_index) = MDP.cumulative_prob(source_state_index,action_index)+prob;                
            end
            [source_index, action_index, cum_prob] = find(MDP.cumulative_prob);
            nEnabledStateAction = length(cum_prob);
            for index = 1:nEnabledStateAction
                if (cum_prob(index)-1)>0.00001
                    source_state = MDP.states(source_index(index));
                    action = MDP.actions(action_index(index));
                    error_msg = "The state action pair ("+source_state+"/"+action+") transitions do not sum to 1.0";
                    error(error_msg);
                end
            end
            %Check if reward list only has unique rewards (no state/action
            %pairs repeated)
            unique_reward = unique(MDP.reward_matrix{1}, 'rows');
            if size(unique_reward, 1) ~= size(MDP.reward_matrix{1},1)
                error("There are repeated state/action pairs in the reward matrix")
            end
            
            MDP.valid = true;
        end
        function set_enabled_actions(MDP)
            %Function that sets the enabled actions attribute, a array of
            %cells, in which each row corresponds to a state, the cell in
            %the first column corresponds to the name of the state, and the
            %cell in the second column corresponds to a list of enabled
            %actions;
            MDP.enabled_actions = cell(MDP.nStates, 2);
            for state_index = 1:MDP.nStates
                state_name = MDP.states(state_index);
                MDP.enabled_actions{state_index, 1} = state_name;
                row = MDP.cumulative_prob(state_index, :);
                action_indices = find(row);
                actions_enabled = [string.empty];
                for index = 1:length(action_indices)
                    action_name = MDP.actions(action_indices(index));
                    actions_enabled = cat(1, actions_enabled, action_name);
                end
                MDP.enabled_actions{state_index, 2} = actions_enabled;
            end
            MDP.prepared = true;
        end
        function actions_enabled = actions_enabled(MDP, state)
            %Function that for a given state, returns a list of the enabled
            %actions;
            state_index = MDP.find_state(state);
            actions_enabled = MDP.enabled_actions{state_index, 2};
        end
        
        function reward = get_reward(MDP, state_index, action_index)
            %index = find(ismember(MDP.reward_matrix{1}, [state_index action_index], 'rows'));
            index = find(all(bsxfun(@eq,MDP.reward_matrix{1},[state_index action_index]),2));
            reward = MDP.reward_matrix{2}(index);
        end
        
        function [end_state_indices, end_state_probs] = action_probs(MDP, state_index, action_index)
            end_state_indices = [];
            end_state_probs = [];
            pos_in_list = find(all(bsxfun(@eq,MDP.transition_matrix{1}(:,1:2),[state_index action_index]),2));
            %pos_in_list = find(ismember(MDP.transition_matrix{1}(:,1:2), [state_index action_index],'rows'));
            nTransitions = size(pos_in_list,1);
            for t_index = 1:nTransitions
                pos = pos_in_list(t_index);
                indices = MDP.transition_matrix{1}(pos,:);
                prob = MDP.transition_matrix{2}(pos);
                end_state_index = indices(3);
                end_state_indices = cat(1, end_state_indices, end_state_index);
                end_state_probs = cat(1, end_state_probs, prob);
            end 
        end
        
        function total_memory = measure_memory(MDP)
            total_memory = 0;
            properties_list = string(properties(MDP));
            nProperties = size(properties_list, 1);
            for p_index = 1:nProperties
                variable = MDP.(properties_list(p_index));
                var_info = whos('variable');
                total_memory = total_memory + var_info.bytes;
            end
            %total memory in MB
            total_memory = total_memory/1024;
        end
            
    end
    
end


