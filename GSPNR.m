classdef GSPNR < handle
    %GSPNR Model for Global Stochastic Petri Net with Rewards
    %   Class that implements GSPNR model
    
    properties (SetAccess = private)
        places = [];            %Array of strings, where each element is a place in the GSPN
        transitions = [];       %Array of strings, where each element is a transition in the GSPN
        type_transitions = [];  %Array of strings, where each element is either the string "imm" or "exp", denoting the type of the corresponding transition in the "transitions" property
        rate_transitions = [];%Array of real numbers, element i corresponds to the weight/rate of transition i
        input_arcs = [];        %Matrix, where the entry (i,j) corresponds to the input arc weight between place i and transition j. If 0, no arc exists
        output_arcs = [];       %Matrix, where the entry (i,j) corresponds to the output arc weight between transition i and place j. If 0, no arc exists
        initial_marking = [];   %Array of positive integers, where each element corresponds to the number of tokens present in the corresponding place in the initial marking.
        current_marking = [];   %Array of positive integers, representing current marking
        place_rewards = [];     %Array of real numbers corresponding to the place rewards for the corresponding place.
        transition_rewards = [];%Array of real numbers corresponding to the transition rewards for the corresponding transition.
    end
    
    methods
        function GSPN = GSPNR()
        end
        function add_places(GSPN, places, ntokens)
            %Adds places to the GSPNR object
            %places is an array of strings, which will be added as places
            %to the GSPNR, ntokens is a vector that represents the initial
            %number of tokens present in the corresponding places to be
            %added.
           if length(places) ~= length(ntokens)
               error('Error, places and ntokens vectors must have the same size')
           end
           GSPN.places = cat(1, GSPN.places, places);
           GSPN.initial_marking = cat(1, GSPN.initial_marking, ntokens);
           GSPN.current_marking = GSPN.initial_marking;
           
           nTransitions = length(GSPN.transitions);
           
           newRow = zeros(length(places), nTransitions);
           GSPN.input_arcs = cat(1, GSPN.input_arcs, newRow);
           
           newColumn = zeros(nTransitions, length(places));
           GSPN.output_arcs = cat(2, GSPN.output_arcs, newColumn);
           GSPN.place_rewards = cat(2, GSPN.place_rewards, zeros(1, length(places)));
        end
        function set_marking(GSPN, new_marking)
            %Sets a new marking to the GSPN object
            nPlaces = length(GSPN.places);
            if nPlaces ~= length(new_marking)
                error("New marking must be consistent with the number of places that exist")
            end
            GSPN.current_marking = new_marking;
        end
        function remove_places(GSPN, places)
            %Removes places from the GSPNR object
           nPlaces = length(places);
           for place = 1:nPlaces
               place_index = find(strcmp(GSPN.places, places(place)));
               GSPN.places(place_index) = [];
               GSPN.initial_marking(place_index) = [];
               GSPN.current_marking(place_index) = [];
               GSPN.input_arcs(place_index,:) = [];
               GSPN.output_arcs(:,place_index) = [];
               GSPN.place_rewards(place_index) = [];
           end
               
        end
        
        function add_transitions(GSPN, transitions, types, rates)
            %Adds immediate and/or exponential transition to the GSPNR
            %object
            if length(transitions) ~= length(types)
                error('Error, transition and type vectors must have the same size')
            end
            GSPN.transitions = cat(2, GSPN.transitions, transitions);
            GSPN.type_transitions = cat(2, GSPN.type_transitions, types);
            GSPN.rate_transitions = cat (2, GSPN.rate_transitions, rates);
            
            nPlaces = length(GSPN.places);
            
            newColumn = zeros(nPlaces, length(transitions));
            GSPN.input_arcs = cat(2, GSPN.input_arcs, newColumn);
            
            newRow = zeros(length(transitions), nPlaces);
            GSPN.output_arcs = cat(1, GSPN.output_arcs, newRow);
            
            GSPN.transition_rewards = cat(2, GSPN.transition_rewards, zeros(1, length(transitions)));
            
        end
        
        function remove_transitions(GSPN, transitions)
            %Removes transitions from input array from the GSPNR object
           nTransitions = length(transitions);
           for transition = 1:nTransitions
               transition_index = find(strcmp(GSPN.transitions, transitions(transition)));
               GSPN.transitions(transition_index) = [];
               GSPN.type_transitions(transition_index) = [];
               GSPN.rate_transitions(transition_index) = [];
               GSPN.input_arcs(:,transition_index) = [];
               GSPN.output_arcs(transition_index, :) = [];
               GSPN.transition_rewards(transition_index) = [];
           end
        end
        
        function add_arcs(GSPN, places, transitions, type, weights)
            %Adds input and output arcs between existing places and transitions to the GSPNR object
           if ~(length(places) == length(transitions) && length(transitions) == length(type) && length(type)== length(weights))
               error('Error, length of inputs not the same')
           end
           nArcs = length(places);
           for arc = 1:nArcs
              place_index = find(strcmp(GSPN.places, places(arc)));
              transition_index = find(strcmp(GSPN.transitions, transitions(arc)));
              if type(arc) == "in"
                  GSPN.input_arcs(place_index,transition_index) = weights(arc);
              elseif type(arc) == "out"
                  GSPN.output_arcs(transition_index, place_index) = weights(arc);
              else
                  error('Name of place or transition cannot be found')
              end
              
           end
        end
        function remove_arcs(GSPN, places, transitions, type)
            %Removes input and/or output arcs from the GSPNR object
           if ~(length(places) == length(transitions) && length(transitions) == length(type))
               error('Error, length of inputs not the same')
           end
           nArcs = length(places);
           for arc = 1:nArcs
               place_index = find(strcmp(GSPN.places, places(arc)));
               transition_index = find(strcmp(GSPN.transitions, transitions(arc)));
               
               if type(arc) == "in"
                   GSPN.input_arcs(place_index, transition_index) = 0;
               elseif type(arc) == "out"
                   GSPN.output_arcs(transition_index, place_index) = 0;
               else
                   error('Name of place or transition cannot be found')
               end
           end
        end
        function set_reward_functions(GSPN, name, reward, type)
            %Sets the place_rewards and transition_rewards properties
            if ( length(name) == length(reward) && length(reward) == length(type) )
                nRewards = length(name);
                for reward_index = 1:nRewards
                    if type(reward_index) == 'place'
                        place_index = find(strcmp(GSPN.places, name(reward_index)));
                        GSPN.place_rewards(place_index) = reward(reward_index);
                    elseif type(reward_index) == 'transition'
                        transition_index = find(strcmp(GSPN.transitions, name(reward_index)));
                        GSPN.transition_rewards(transition_index) = reward(reward_index);
                    else
                        error('Type of reward must be either "place" or "transition"')
                    end
                end
            else
                error('Length of the input vectors must be consistent')
            end
        end
        function [enabled_imm_trans, enabled_exp_trans] = enabled_transitions(GSPN)
            %Returns a string array of the enabled transitions in the current marking
           enabled_imm_trans = [];
           enabled_exp_trans = [];
           nTransitions = length(GSPN.transitions);
           for transition_index = 1:nTransitions
               intermediate_marking = GSPN.current_marking - GSPN.input_arcs(:,transition_index)';
               if (any(intermediate_marking<0) )
                   continue
               else
                   if GSPN.type_transitions(transition_index) == "imm"
                       enabled_imm_trans = cat(2, enabled_imm_trans, GSPN.transitions(transition_index));
                   else
                       enabled_exp_trans = cat(2, enabled_exp_trans, GSPN.transitions(transition_index));
                   end
               end
           end
        end
        
        function fire_transition(GSPN, transition)
            %Fires the transition in the input and updates the current marking property
           transition_index = find(strcmp(GSPN.transitions, transition));
           intermediate_marking = GSPN.current_marking - GSPN.input_arcs(:,transition_index)';
           if ( any(intermediate_marking<0) )
              error('Transition was not enabled and so cannot fire') 
           end
           final_marking = intermediate_marking + GSPN.output_arcs(transition_index,:);
           GSPN.current_marking = final_marking;
        end
        
        function emb_MDP = ConverttoMDP(GSPN)
           %Transforms the GSPNR object into the equivalent MDP
           %Markings are added to the state as soon as they are discovered
           emb_MDP = MarkovDecisionProblem()
           covered_marking_list = [];   %The marking represented by row i, is
           covered_state_list = [];     %is equivalent to state in row i of this vector
           markings_to_explore = [GSPN.initial_marking];
           state_index = 1;
           state_name = "S"+string(state_index);
           state_index = state_index + 1;
           covered_marking_list = cat(1,covered_marking_list,GSPN.initial_marking)
           covered_state_list = cat(1, covered_state_list, state_name)
           emb_MDP.add_state(state_name);
           while ~(isempty(markings_to_explore))
              original_marking = markings_to_explore(1,:)
              GSPN.set_marking(original_marking);
              [exists, original_state_index] = ismember(original_marking, covered_marking_list, 'rows');
              original_state = "S" + string(original_state_index);
              
              [imm_enabled, exp_enabled] = GSPN.enabled_transitions();
              
              nTransitions = length(imm_enabled);
              if (~isempty(imm_enabled))
                weight_sum = 0;
                imm_enabled_race = [string.empty];
                for imm_enabled_index = 1:nTransitions
                    transition = imm_enabled(imm_enabled_index);
                    transition_index = find(GSPN.transitions == transition);
                    rate = GSPN.rate_transitions(transition_index);
                    if rate>0
                        imm_enabled_race = cat(2, imm_enabled_race, transition);
                    end
                    weight_sum = weight_sum + rate;
                end
                race_name = "RACE_" + strjoin(imm_enabled_race,"_")
              end
              
              if (isempty(exp_enabled))
                  %Probabilistic state
                  markings_to_explore;
                  nTransitions = length(imm_enabled);
                  for imm_enabled_index = 1:nTransitions
                      transition = imm_enabled(imm_enabled_index);
                      GSPN.fire_transition(transition);
                      new_marking = GSPN.current_marking;
                      [explored, target_state_index] = ismember(new_marking, covered_marking_list, 'rows');
                      if (~explored)
                          markings_to_explore = cat(1, markings_to_explore, new_marking);
                          covered_marking_list = cat(1, covered_marking_list, new_marking);
                          target_state_name = "S" + string(state_index);
                          covered_state_list = cat(1, covered_state_list, target_state_name);
                          target_state_index = size(covered_state_list, 1);
                          emb_MDP.add_state(target_state_name);
                          state_index = state_index + 1;
                      end
                      if weight_sum ~= 0
                          %RACE CONDITION
                          transition_index = find(GSPN.transitions == transition);
                          action_index = emb_MDP.add_action(race_name, "imm");
                          transition_weight = GSPN.rate_transitions(transition_index);
                          emb_MDP.transition_matrix(original_state_index, action_index, target_state_index) = transition_weight/weight_sum; 
                      else
                          %CONTROLLABLE ACTION
                          transition_index = find(GSPN.transitions == transition);
                          transition;
                          action_index = emb_MDP.add_action(transition, "imm");
                          emb_MDP.transition_matrix(original_state_index, action_index, target_state_index) = 1.0;
                          
                      end
                      original_marking;
                      GSPN.set_marking(original_marking);
                  end
                  
              elseif(isempty(imm_enabled)&&~isempty(exp_enabled))
                  %Markovian state
                  nTransitions = length(exp_enabled);
                  for exp_enabled_index = 1:nTransitions
                     transition = exp_enabled(exp_enabled_index)
                     GSPN.fire_transition(transition);
                     new_marking = GSPN.current_marking;
                     [explored, target_state_index] = ismember(new_marking, covered_marking_list, 'rows')
                     if (~explored)
                         markings_to_explore = cat(1, markings_to_explore, new_marking);
                         covered_marking_list = cat(1, covered_marking_list, new_marking);
                         target_state_name = "S" + string(state_index);
                         covered_state_list = cat(1, covered_state_list, target_state_name);
                         target_state_index = size(covered_state_list, 1);
                         emb_MDP.add_state(target_state_name);
                         state_index = state_index + 1;
                     end
                     transition_index = find(GSPN.transitions == transition);
                     transition_rate = GSPN.rate_transitions(transition_index);
                     transition_name = transition+"EXP";
                     action_index = emb_MDP.add_action(transition_name, "exp");
                     emb_MDP.exponential_transition_matrix(original_state_index, action_index, target_state_index) = transition_rate;
                     GSPN.set_marking(original_marking);
                  end
                  
              else
                  %Hybrid state
                  
              end
              markings_to_explore(1,:) = [];
           end
           covered_marking_list
           covered_state_list
        end
    end
    
end


