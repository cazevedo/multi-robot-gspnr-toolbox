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
