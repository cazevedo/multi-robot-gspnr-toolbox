function simulate_policy(obj,policy, step_time)
%EVALUATE_POLICY Simulate the policy execution in non real time
%   Detailed explanation goes here

    %Build easier policy struct
    markings_to_transition = struct();
    markings_to_transition.markings = policy.state_index_to_markings;
    for m_index = 1:size(policy.state_index_to_markings, 1)
        state_name = policy.states(m_index);
        state_index = policy.mdp.find_state(state_name);
        action_index = policy.mdp_policy(state_index);
        markings_to_transition.transitions(m_index) = policy.mdp.actions(action_index);
    end
    disp("Finished loading policy");
    done = 0;
    %Start Executing
    while(~done)
        if step_time == false
            pause();
        else
            pause(step_time);
        end
        %Print current marking
        marking_type = obj.check_marking_type();
        marking = obj.current_marking;
        disp("-----------------------------")
        disp("Current marked places are the following:");
        marked_place_indices = find(marking);
        for p_ii = 1:size(marked_place_indices, 2)
            place_index = marked_place_indices(p_ii);
            place_name = obj.places(place_index);
            ntokens = obj.current_marking(place_index);
            msg = place_name+":"+string(ntokens);
            disp(msg);
        end
        disp("-----------------------------")
        [imm, exp] = obj.enabled_transitions();
        if (marking_type == "DET" || marking_type == "RAN")
            if marking_type == "DET"
                %Check policy
                [exists, marking_index] = ismember(marking, markings_to_transition.markings, 'rows');
                if exists
                    transition = markings_to_transition.transitions(marking_index);
                    msg = "Will fire transition from policy - "+transition;
                    disp(msg);
                    if ~isempty(find(imm == transition))
                        %Fire transition chosen by policy
                        obj.fire_transition(transition)
                        msg = "Fired transition from policy - "+transition;
                        disp(msg);
                    else
                        error("Transition given by policy was not enabled")
                    end
                else
                    error("Partial policy");
                end
                continue;
            end
            if marking_type == "RAN"
                %Get weights of all enabled imm transitions
                %Sample uniform distribution according to weights
                %Fire transition chosen by sample
                nImm = size(imm, 2);
                weights = zeros(1, nImm);
                for index = 1:nImm
                    transition = imm(index);
                    trans_index = obj.find_transition_index(transition);
                    weight = obj.rate_transitions(trans_index);
                    weights(index) = weight;
                end
                norm_constant = sum(weights);
                weights = weights./norm_constant;
                transition_chosen = randsample(imm, 1, true, weights);
                msg = "Fired transition from random switch - "+transition_chosen;
                disp(msg);
                obj.fire_transition(transition_chosen);
                continue;
            end
        elseif marking_type == "TAN"
            %Sample exponential distributions
            nExp = size(exp, 2);
            delays = zeros(1, nExp);
            for index = 1:nExp
                transition = exp(index);
                trans_index = obj.find_transition_index(transition);
                rate = obj.rate_transitions(trans_index);
                delay(index) = round(exprnd(1/rate), 2);
            end
            %Choose faster one
            [chosen_delay, chosen_trans_index] = min(delays, [], 2);
            transition_chosen = exp(chosen_trans_index);
            %Fire transition
            msg = "Will fire transition from race condition - "+transition_chosen;
            disp(msg);
            obj.fire_transition(transition_chosen);
        end
    end
end

