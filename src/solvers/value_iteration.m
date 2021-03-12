function [values, policy] = value_iteration(MDP, max_min, gamma, epsilon)

    if ~(MDP.valid && MDP.prepared)
        error("MDP must be valid and prepared to run value iteration")
    end

    values = zeros(1, MDP.nStates);
    policy = zeros(1, MDP.nStates);
    
    if max_min
        Q_max = -Inf(1, MDP.nStates);
    else
        Q_max = Inf(1, MDP.nStates);
    end
    
    max_res = -Inf;
    converged = false;
    step = 0;
    while (~converged)
        step = step + 1;
        for state_index = 1:MDP.nStates
            old_value = values(state_index);
            [new_value, new_policy] = bellman_update(MDP, state_index, max_min, gamma, values, Q_max);
            new_res = abs(new_value - old_value);
            values(state_index) = new_value;
            if new_policy ~= -1
                policy(state_index) = new_policy;
            end
            Q_max(state_index) = new_value;
            if new_res>max_res
                max_res = new_res;
            end
        end
        converged = max_res<epsilon;
        max_res = -Inf;
        values
        policy
    end
    debug = "Number of iterations done: "+string(step)
end

function [new_value, new_policy] = bellman_update(MDP, state_index, max_min, gamma, values, Q_max)

    if max_min
        isbetter = @(x,y)x>y;
    else
        isbetter = @(x,y)x<y;
    end
    
    new_policy = -1;

    state_name = MDP.states(state_index);
    enabled_actions = MDP.actions_enabled(state_name);
    nEnabledActions = length(enabled_actions);
    
    for a_index = 1:nEnabledActions
        action_name = enabled_actions(a_index);
        action_index = MDP.find_action(action_name, "imm");
        
        reward = MDP.get_reward(state_index, action_index);
        if reward ~= 0
            Q_val = reward;
        else
            Q_val = 0;
        end
        
        [end_state_indices, end_state_probs] = MDP.action_probs(state_index, action_index);
        
        nEndStates = length(end_state_indices);
        
        for s_index = 1:nEndStates
            Q_val = Q_val + (gamma * end_state_probs(s_index) * values(end_state_indices(s_index)) );
        end
        
        if isbetter(Q_val, Q_max(state_index))
            Q_max(state_index) = Q_val;
            new_policy = action_index;
        end
    end
    new_value = Q_max(state_index);
    new_policy;
    %debug = "Bellman Update for state S"+string(state_index)+" new value is "+string(new_value)
    
    if nEnabledActions == 0
        new_policy = 0;
        new_value = 0;
    end
end
