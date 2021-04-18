classdef GSPNR < matlab.mixin.Copyable
    %GSPNR Model for Global Stochastic Petri Net with Rewards
    %   Class that implements GSPNR model
    
    properties (SetAccess = protected)
        places = [];            %Array of strings, where each element is a place in the GSPN
        transitions = [];       %Array of strings, where each element is a transition in the GSPN
        type_transitions = [];  %Array of strings, where each element is either the string "imm" or "exp", denoting the type of the corresponding transition in the "transitions" property
        rate_transitions = [];%Array of real numbers, element i corresponds to the weight/rate of transition i
        input_arcs = [];        %Matrix, where the entry (i,j) corresponds to the input arc weight between place i and transition j. If 0, no arc exists
        output_arcs = [];       %Matrix, where the entry (i,j) corresponds to the output arc weight between transition i and place j. If 0, no arc exists
        arcs = struct();        %Struct that holds arcs auxiliary variables
        initial_marking = [];   %Array of positive integers, where each element corresponds to the number of tokens present in the corresponding place in the initial marking.
        current_marking = [];   %Array of positive integers, representing current marking
        place_rewards = [];     %Array of real numbers corresponding to the place rewards for the corresponding place.
        transition_rewards = [];%Array of real numbers corresponding to the transition rewards for the corresponding transition.
        %Add coment
    end
    
    methods
        function GSPN = GSPNR()
            GSPN.arcs.places = [string.empty];
            GSPN.arcs.transitions = [string.empty];
            GSPN.arcs.types = [string.empty];
            GSPN.arcs.weights = [];
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
           GSPN.places = cat(2, GSPN.places, places);
           GSPN.initial_marking = cat(2, GSPN.initial_marking, ntokens);
           GSPN.current_marking = GSPN.initial_marking;
           
           nTransitions = length(GSPN.transitions);
           
           newRow = zeros(length(places), nTransitions);
           GSPN.input_arcs = cat(1, GSPN.input_arcs, newRow);
           
           newColumn = zeros(nTransitions, length(places));
           GSPN.output_arcs = cat(2, GSPN.output_arcs, newColumn);
           GSPN.place_rewards = cat(2, GSPN.place_rewards, zeros(1, length(places)));
        end
        function index = find_place_index(GSPN, place_name)
            index = find(GSPN.places == place_name);
            if isempty(index)
                warning("Could not find place, returned 0");
                index = 0;
            end
        end
        function set_marking(GSPN, new_marking)
            %Sets a new marking to the GSPN object
            nPlaces = length(GSPN.places);
            if nPlaces ~= length(new_marking)
                error("New marking must be consistent with the number of places that exist")
            end
            GSPN.current_marking = new_marking;
        end
        function type = check_marking_type(GSPN)
            %Returns either "TAN", "DET" or "RAN" if the current marking is
            %either tangible, deterministic (expecting policy action), or
            %expecting a random switch, respectively.
            [imm, exp] = GSPN.enabled_transitions();
            nTransitions = length(imm);
            if (~isempty(imm))
                weight_sum = 0;
                type = "RAN";
                for imm_enabled_index = 1:nTransitions
                    transition = imm(imm_enabled_index);
                    transition_index = find(GSPN.transitions == transition);
                    rate = GSPN.rate_transitions(transition_index);
                    if rate == 0
                        type = "DET";
                        break
                    end
                    weight_sum = weight_sum + rate;
                end
                  
            elseif( isempty(imm) && ~isempty(exp) )
                  type = "TAN";
            elseif( isempty(imm) && isempty(exp) )
                  type = "SINK";
            end
            
            
            
        end
        function remove_places(GSPN, places)
            %Removes places from the GSPNR object
           nPlaces = length(places);
           arc_places = [string.empty];
           arc_transitions = [string.empty];
           arc_types = [];
           for p_index = 1:nPlaces
               place_name = places(p_index);
               place_index = find(strcmp(GSPN.places, place_name));
               target_trans_indices = find(GSPN.input_arcs(place_index, :));
               target_trans_names = translate_to_names(GSPN.transitions, target_trans_indices);
               nTargetTransNames = size(target_trans_indices, 2);
               arc_places = cat(2, arc_places, repmat(place_name, [1 nTargetTransNames]));
               arc_transitions = cat(2, arc_transitions, target_trans_names);
               arc_types = cat(2, arc_types, repmat("in", [1 nTargetTransNames]));
               source_trans_indices = find(GSPN.output_arcs(:, place_index));
               source_trans_names = translate_to_names(GSPN.transitions, source_trans_indices');
               nSourceTransNames = size(source_trans_names, 2);
               arc_places = cat(2, arc_places, repmat(place_name, [1 nSourceTransNames]));
               arc_transitions = cat(2, arc_transitions, source_trans_names);
               arc_types = cat(2, arc_types, repmat("out", [1 nSourceTransNames]));
           end
           GSPN.remove_arcs(arc_places, arc_transitions, arc_types);
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
        function index = find_transition_index(GSPN, trans_name)
            index = find(GSPN.transitions == trans_name);
            if isempty(index)
                warning("Could not find place, returned 0");
                index = 0;
            end
        end
        
        function remove_transitions(GSPN, transitions)
           nTransitions = length(transitions);
           arc_places = [string.empty];
           arc_transitions = [string.empty];
           arc_types = [];
           for t_index = 1:nTransitions
               transition_name = transitions(t_index);
               transition_index = find(strcmp(GSPN.transitions, transition_name));
               %Saving input arcs that need to be deleted
               input_place_indices = find(GSPN.input_arcs(:,transition_index));
               input_place_names = translate_to_names(GSPN.places, input_place_indices');
               nInputPlaceIndices = size(input_place_names, 2);
               arc_places = cat(2, arc_places, input_place_names);
               arc_transitions = cat(2, arc_transitions, repmat(transition_name, [1 nInputPlaceIndices]));
               arc_types = cat(2, arc_types, repmat("in", [1 nInputPlaceIndices]));
               %Doing the same thing but for output arcs
               output_place_indices = find(GSPN.output_arcs(transition_index,:));
               output_place_names = translate_to_names(GSPN.places, output_place_indices);
               nOutputPlaceIndices = size(output_place_names, 2);
               arc_places = cat(2, arc_places, output_place_names);
               arc_transitions = cat(2, arc_transitions, repmat(transition_name, [1 nOutputPlaceIndices]));
               arc_types = cat(2, arc_types, repmat("out", [1 nOutputPlaceIndices]));
           end
           GSPN.remove_arcs(arc_places, arc_transitions, arc_types);
               
           %Removes transitions from input array from the GSPNR object
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
        
        function [exp_trans, exp_trans_indices] = get_exponential_transitions(GSPN)
            %Returns list of exponential transition and their corresponding
            %indices
            exp_trans_indices = find(GSPN.type_transitions == "exp");
            nEXPTrans = size(exp_trans_indices, 2);
            for t_index = 1:nEXPTrans
                exp_trans(t_index) = GSPN.transitions(exp_trans_indices(t_index));
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
           GSPN.arcs.places = cat(2, GSPN.arcs.places, places);
           GSPN.arcs.transitions = cat(2, GSPN.arcs.transitions, transitions);
           GSPN.arcs.types = cat(2, GSPN.arcs.types, type);
           GSPN.arcs.weights = cat(2, GSPN.arcs.weights, weights);
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
               p_index = find(GSPN.arcs.places == places(arc));
               t_index = find(GSPN.arcs.transitions == transitions(arc));
               ty_index = find(GSPN.arcs.types == type(arc));
               index = intersect(p_index, t_index);
               index = intersect(index, ty_index);
               GSPN.arcs.places(index) = [];
               GSPN.arcs.transitions(index) = [];
               GSPN.arcs.types(index) = [];
               GSPN.arcs.weights(index) = [];
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
        
        function target_trans = find_target_trans(GSPN, place)
            place_index = GSPN.find_place_index(place);
            arc_row_vector = GSPN.input_arcs(place_index, :);
            trans_indices = find(arc_row_vector);
            target_trans = translate_to_names(GSPN.transitions, trans_indices);
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
        
        function [emb_MDP, covered_marking_list, covered_state_list, covered_state_type] = toMDP(GSPN)
           %Transforms the GSPNR object into the equivalent MDP
           %Markings are added to the state as soon as they are discovered
           emb_MDP = MDP();
           covered_marking_list = [];   %The marking represented by row i, is
           covered_state_list = [];     %is equivalent to state in row i of this vector
           covered_state_type = [string.empty];
           
           %Initialization with the initial state equivalent to the initial
           %marking of the GSPNR
           markings_to_explore = [GSPN.initial_marking];
           state_index = 1;
           state_name = "S"+string(state_index);
           state_index = state_index + 1;
           covered_marking_list = cat(1,covered_marking_list,GSPN.initial_marking);
           covered_state_list = cat(1, covered_state_list, state_name);
           emb_MDP.add_state(state_name);
           
           while ~(isempty(markings_to_explore))
              original_marking = markings_to_explore(1,:); %Marking to be explored
%               log = "Exploring marking - "
%               original_marking
              GSPN.set_marking(original_marking);
              [exists, original_state_index] = ismember(original_marking, covered_marking_list, 'rows');
              original_state = covered_state_list(original_state_index);
              
              [imm_enabled, exp_enabled] = GSPN.enabled_transitions(); %Enabled transitions in this state
              
              if (~isempty(imm_enabled))
                  covered_state_type(original_state_index) = "VAN";
              elseif( isempty(imm_enabled) && ~isempty(exp_enabled) )
                  covered_state_type(original_state_index) = "TAN";
              elseif( isempty(imm_enabled) && isempty(exp_enabled) )
                  covered_state_type(original_state_index) = "SINK";
              end
              %Checking for and creating name of race conditions between
              %immediate transitions
              nTransitions = length(imm_enabled);
              if (~isempty(imm_enabled))
                weight_sum = 0;
                imm_enabled_race = [string.empty];%Transitions that are part of a race condition (any that has weight~=0)
                for imm_enabled_index = 1:nTransitions
                    transition = imm_enabled(imm_enabled_index);
                    transition_index = find(GSPN.transitions == transition);
                    rate = GSPN.rate_transitions(transition_index);
                    if rate>0
                        imm_enabled_race = cat(2, imm_enabled_race, transition);
                    end
                    weight_sum = weight_sum + rate;
                end
                race_name = "RACE_" + strjoin(imm_enabled_race,"_");
              end
              probabilistic_state = false;
              
              %Firing any immediate transitions if they are enabled
              nTransitions = length(imm_enabled);
              for imm_enabled_index = 1:nTransitions
                  probabilistic_state = true; %If there are any immediate transitions to fire, this will execute
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
                  else
                      target_state_name = emb_MDP.states(target_state_index);
                  end
                  if weight_sum ~= 0
                      %RACE CONDITION
                      transition_index = find(GSPN.transitions == transition);
                      action_index = emb_MDP.add_action(race_name, "imm");
                      transition_weight = GSPN.rate_transitions(transition_index);
                      emb_MDP.set_transition(original_state, race_name, target_state_name, transition_weight/weight_sum, "imm"); 
                  else
                      %CONTROLLABLE ACTION
                      transition_index = find(GSPN.transitions == transition);
                      action_index = emb_MDP.add_action(transition, "imm");
                      emb_MDP.set_transition(original_state, transition, target_state_name, 1.0, "imm");
                      reward = GSPN.transition_rewards(transition_index);  
                      if (reward ~= 0)
                          emb_MDP.set_reward(original_state, transition, reward);
                      end

                  end
                  GSPN.set_marking(original_marking);
              end
              
              if (probabilistic_state && ~isempty(exp_enabled) )
                  %Hybrid state, where there are immediate and exponential
                  %enabled transitions
                  target_state = "WAIT_"+string(covered_state_list(original_state_index));
                  covered_marking_list = cat(1, covered_marking_list, original_marking);
                  covered_state_list = cat(1, covered_state_list, target_state);
                  
                  wait_state_index = emb_MDP.add_state(target_state);
                  wait_action_index = emb_MDP.add_action("WAIT","imm");
                  covered_state_type(wait_state_index) = "TAN";
                  %Adding action from original state to copy "wait" state;
                  emb_MDP.set_transition(original_state, "WAIT", target_state, 1.0, "imm");
                  %All exponential transitions that follow, in the MDP
                  %begin in this dummy "wait" state, and not the original
                  %one
                  original_state = target_state;
                  original_state_index = wait_state_index;
                  
              end
              
              %Fire any exponential transitions, will not run anything if
              %list is empty
              nTransitions = length(exp_enabled);
              for exp_enabled_index = 1:nTransitions
                 transition = exp_enabled(exp_enabled_index);
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
                 else
                     target_state_name = covered_state_list(target_state_index);
                 end
                 transition_index = find(GSPN.transitions == transition);
                 transition_rate = GSPN.rate_transitions(transition_index);
                 transition_name = transition+"EXP";
                 action_index = emb_MDP.add_action(transition_name, "exp");
                 %TODO Change to use names, not index
                 emb_MDP.set_transition(original_state, transition_name, target_state_name, transition_rate, "exp");
                 GSPN.set_marking(original_marking);
              end
              
              markings_to_explore(1,:) = [];
           end
           
           emb_MDP.consolidation_uniformization(covered_state_type);
           
           exp_action_index = emb_MDP.find_action("EXP", "imm");
           for state = 1:emb_MDP.nStates
              state_name = covered_state_list(state);
              state_index = emb_MDP.find_state(state_name);
              state_type = covered_state_type(state);
              if state_index == 0
                  error("Something went very wrong")
              end                  
              equivalent_marking = covered_marking_list(state,:);
              marked_places = equivalent_marking~=0;
              state_reward = dot(GSPN.place_rewards,marked_places);
              if state_type == "TAN"
                normalized_reward = state_reward/emb_MDP.eta;
                emb_MDP.set_reward(state_name, "EXP", normalized_reward);
              end
           end
           
        end
        function format(GSPN,argument_list)
            nArguments = size(argument_list, 2);
            for blank_index = 1:nArguments
                old_tag = "<"+string(blank_index)+">";
                GSPN.places = strrep(GSPN.places, old_tag, argument_list(blank_index));
                GSPN.transitions = strrep(GSPN.transitions, old_tag, argument_list(blank_index));
                GSPN.arcs.places = strrep(GSPN.arcs.places, old_tag, argument_list(blank_index));
                GSPN.arcs.transitions = strrep(GSPN.arcs.transitions, old_tag, argument_list(blank_index));
            end
        end
    end
    
end


