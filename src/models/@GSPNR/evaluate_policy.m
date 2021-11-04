function results = evaluate_policy(obj, policy, max_time, n_report, handcrafted)
%EVALUATE_POLICY Simulate the policy execution with simulated time
%Input:
%   policy_struct   [struct] - struct containing the following fields:
%                       * state_index_to_markings - matrix where each row is a GSPNR marking, and the index corresponds
%                       to the index of the corresponding state of the equivalent MDP model;
%                       * states - string array, where the elements are ordered according to their corresponding index
%                       * mdp_policy - string array, where each element corresponds to the transition that should fire
%                       when the MDP model is in the state of the element's index;
%                       * mdp - MDP object where policy was computed on;
%   nTransitions    [int] - number of transitions to fire;
%   n_report        [int] - function reports to command prompt every "n_report" transitions fired
%Output:
%   results         [struct] - containing the following fields:
%                       * markings - matrix where each row represents a marking;
%                       * transitions - string array where each element is
%                       the transition fired that transitioned the GSPNR
%                       into the marking in the same position in the
%                       "markings" field;
%                       * timestamps - duration array, where each element
%                       represents the duration between when the simulation
%                       started and the time that the corresponding (in the
%                       same position in the "transitions" field)
%                       transition fired;

    if ~isempty(policy)
        if handcrafted == 0
        %Build easier policy struct
            markings_to_transition = struct();
            markings_to_transition.markings = policy.state_index_to_markings;
            for m_index = 1:size(policy.state_index_to_markings, 1)
                state_name = policy.states(m_index);
                state_index = policy.mdp.find_state(state_name);
                action_index = policy.mdp_policy(state_index);
                markings_to_transition.transitions(m_index) = policy.mdp.actions(action_index);
            end
        else
            markings_to_transition = struct();
            markings_to_transition.markings = policy.markings;
            for m_index = 1:size(policy.markings, 1)
                markings_to_transition.transitions(m_index) = policy.transitions(m_index);
            end
        end
            
    

        disp("Finished loading policy");
    else
        markings_to_transition.markings = zeros(1, size(obj.places, 2));
        markings_to_transition.transition = "";
    end
    
    %Setting initial marking as current marking
    obj.set_marking(obj.initial_marking);
    
    %Preparing results array
    start = datetime('now');
    now = start;
    reporting_time = start;
    results = struct("markings", [], "transitions", [], "timestamps", [], "reward", 0);
    
    transitions_fired = 0;
    policy_decisions = 0;
    random_decisions = 0;
    done = 0;
    %Start Executing
    while(~done)
        %Print current marking
        marking_type = obj.check_marking_type();
        marking = obj.current_marking;
        if mod(transitions_fired, n_report) == 0 && transitions_fired ~=0
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
            disp(msg)
            previous_reporting_time = reporting_time;
            reporting_time = datetime('now');
            msg = "Time taken to fire "+string(n_report)+" transitions is "+string(seconds(reporting_time-previous_reporting_time))+" seconds";
            disp(msg);
            disp("-----------------------------")
        end
        [imm, exp] = obj.enabled_transitions();
        if marking_type == "SINK"
            done = 1;
        end
        if (marking_type == "DET" || marking_type == "RAN")
            if marking_type == "DET"
                %Check policy
                [exists, marking_index] = ismember(marking, markings_to_transition.markings, 'rows');
                if exists == 0
                    transition="";
                else
                    transition = markings_to_transition.transitions(marking_index);
                end
                if exists && transition ~=""
                    %msg = "Will fire transition from policy - "+transition;
                    %disp(msg);
                    if ~isempty(find(imm == transition))
                        %Fire transition chosen by policy
                        obj.fire_transition(transition)
                        %Adding transition reward
                        trans_reward = obj.transition_rewards(obj.find_transition_index(transition));
                        results.reward = results.reward + trans_reward;
                        % Saving results
                        timestamp = now-start;
                        marking = obj.current_marking;
                        results.markings = cat(1, results.markings, marking);
                        results.transitions = cat(1, results.transitions, transition);
                        results.timestamps = cat(1, results.timestamps, timestamp);
                        
                        %msg = "Fired transition from policy - "+transition;
                        %disp(msg);
                        
                        transitions_fired = transitions_fired + 1;
                        policy_decisions = policy_decisions + 1;
                    else
                        error("Transition given by policy was not enabled")
                    end
                else
                    nImmTransitions = size(imm, 2);
                    rn_trans = randi(nImmTransitions);
                    transition = imm(rn_trans);
                    obj.fire_transition(transition);
                    trans_reward = obj.transition_rewards(obj.find_transition_index(transition));
                    results.reward = results.reward + trans_reward;
                    % Saving results
                    timestamp = now-start;
                    marking = obj.current_marking;
                    results.markings = cat(1, results.markings, marking);
                    results.transitions = cat(1, results.transitions, transition);
                    results.timestamps = cat(1, results.timestamps, timestamp);
                    transitions_fired = transitions_fired + 1;
                    random_decisions = random_decisions + 1;
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
                
                %msg = "Fired transition from random switch - "+transition_chosen;
                %disp(msg);
                
                obj.fire_transition(transition_chosen);
                %Adding transition reward
                trans_reward = obj.transition_rewards(obj.find_transition_index(transition_chosen));
                results.reward = results.reward + trans_reward;
                %Saving results
                timestamp = now-start;
                marking = obj.current_marking;
                results.markings = cat(1, results.markings, marking);
                results.transitions = cat(1, results.transitions, transition);
                results.timestamps = cat(1, results.timestamps, timestamp);
                transitions_fired = transitions_fired + 1;
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
                delays(index) = round(exprnd(1/rate), 2);
            end
            %Choose faster one
            [chosen_delay, chosen_trans_index] = min(delays, [], 2);
            transition_chosen = exp(chosen_trans_index);
            %Fire transition
            
            %msg = "Will fire transition from race condition - "+transition_chosen+"in "+chosen_delay+"s";
            %disp(msg);
            
            %pause(chosen_delay);
            now = now + duration(0,0,chosen_delay);
            
            obj.fire_transition(transition_chosen);
            %Add place reward
            marked_places = obj.current_marking~=0;
            results.reward = (dot(obj.place_rewards,marked_places) * chosen_delay) + results.reward;
            %Saving results
            timestamp = now-start;
            marking = obj.current_marking;
            results.markings = cat(1, results.markings, marking);
            results.transitions = cat(1, results.transitions, transition_chosen);
            results.timestamps = cat(1, results.timestamps, timestamp);
            transitions_fired = transitions_fired + 1;
            if seconds(timestamp) > max_time
                done = 1;
                disp(["Policy decisions - " policy_decisions]);
                disp(["Random decisions - " random_decisions]);

            end
            if mod(transitions_fired, n_report) == 0
                msg = "Simulated time is "+string(seconds(results.timestamps(end)))+"seconds";
                disp(msg);
            end
        end
    end
end

