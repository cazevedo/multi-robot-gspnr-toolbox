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
        transition_matrix = {};
        nTransitions = 0;
        exponential_transition_matrix = {};
        nEXPTransitions = 0;
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
        
        function action_index = find_action(MDP, action_name)
            index = find(MDP.actions == action_name);
            if isempty(index)
                index = 0;
                disp("Action does not exist")
            else
                action_index = index;
            end
        end
        
        function set_transition(MDP, source_state, action, end_state, prob, type)
            source_state_index = MDP.find_state(source_state);
            action_index = MDP.find_action(action);
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
            action_index = MDP.find_action(action);
            
            if ~(state_index && action_index)
                error("Tried to add reward to nonexistent element");
            else
                indices = [state_index, action_index];
                MDP.nRewards = MDP.nRewards + 1;
                MDP.reward_matrix{MDP.nRewards, 1} = indices;
                MDP.reward_matrix{MDP.nRewards, 2} = reward;
            end
        end
                
        
        function consolidation_uniformization(MDP)
           %Calculation of eta, uniformization constant
           exp_action_matrix = zeros(MDP.nStates, MDP.nStates);
           total_trans_freq = zeros(MDP.nStates, MDP.nStates);
           exit_rates = [];
%            total_trans_freq = sum(exp_action_matrix, 2)
%            total_trans_freq = reshape(total_trans_freq, MDP.nStates, MDP.nStates);
           for state_index = 1:MDP.nStates
              total_trans_freq(state_index,:) = sum(MDP.exponential_transition_matrix(state_index,:,:),2);
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
                       exp_action_matrix(source_state, target_state) = 1 - (exit_rates(source_state)-total_trans_freq(source_state,target_state))*(1/eta);
                   else
                       %Nondiagonal, normal expression
                       exp_action_matrix(source_state, target_state) = total_trans_freq(source_state,target_state)/eta;
                   end
               end
           end
            
           exponential_action = MDP.add_action("EXP","imm");
           MDP.transition_matrix(:,exponential_action,:) = exp_action_matrix;
           MDP.exponential_transition_matrix = [];
           MDP.nEXPActions = 0;
           MDP.exp_actions = [string.empty];
        end
        
        function policy = value_iteration(MDP, max_min, gamma, epsilon)
            
            utility = zeros(1, MDP.nStates);
            policy = zeros(1, MDP.nStates);
            max_res = -Inf;
            converged = false;
            
            while (~converged)
                for state_index = 1:MDP.nStates
                    old_value = utility(state_index);
                    state_name = MDP.states(state_index);
                    [new_value, new_value] = bellman_update(MDP, state_name, max_min, utility);
                    new_res = abs(new_value - old_value);
                    utility(state_index) = new_value;
                    if new_res>max_res
                        max_res = new_res;
                    end
                end
                converged = max_res<epsilon;
                max_res = -Inf;
            end
        end
                
    end
    
end


