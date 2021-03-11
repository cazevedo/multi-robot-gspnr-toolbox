function [values, policy] = value_iteration(MDP, max_min, gamma, epsilon)
            
    values = zeros(1, MDP.nStates);
    policy = zeros(1, MDP.nStates);
    max_res = -Inf;
    converged = false;

    while (~converged)
        for state_index = 1:MDP.nStates
            old_value = values(state_index);
            [new_value, new_policy] = bellman_update(MDP, state_index, max_min, gamma, values);
            new_res = abs(new_value - old_value);
            values(state_index) = new_value;
            policy(state_index) = new_policy;
            if new_res>max_res
                max_res = new_res;
            end
        end
        converged = max_res<epsilon;
        max_res = -Inf;
    end
end

function [new_value, new_policy] = bellman_update(MDP, state_index, max_min, gamma, values)

    if max_min
        isbetter = @(x,y)x>y;
        Q_max = -Inf;
    else
        isbetter = @(x,y)x<y;
        Q_max = Inf;
    end

    state_name = MDP.states(state_index);
    enabled_actions = MDP.actions_enabled(state_name);
    nEnabledActions = length(enabled_actions);
    
    for index = 1:nEnabledActions
        action_name = enabled_actions(index);
        action_index = MDP.find_action(action_name, "imm");
        
        reward = MDP.get_reward(state_index, action_index);
        if reward ~= 0
            Q_val = reward;
        else
            Q_val = 0;
        end
        
        [end_state_indices, end_state_probs] = MDP.action_probs(state_index, action_index);
        
        nEndStates = length(end_state_indices);
        
        for index = 1:nEndStates
            Q_val = Q_val + (gamma * end_state_probs(index) * values(end_state_indices(index)) );
        end
        
        if isbetter(Q_val, Q_max)
            Q_max = Q_val;
            new_policy = action_index;
        end
    end
    new_value = Q_max;
        
    if nEnabledActions == 0
        new_policy = 0;
        new_value = 0;
    end
end
