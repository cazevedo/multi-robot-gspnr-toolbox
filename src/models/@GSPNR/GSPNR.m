classdef GSPNR < matlab.mixin.Copyable
    %GSPNR Model for Global Stochastic Petri Net with Rewards
    %   Class that implements GSPNR model
    
    properties (SetAccess = protected)
        places = [];            %[string array] - each element is a place in the GSPN
        transitions = [];       %[string array] - each element is a transition in the GSPN
        type_transitions = [];  %[string array] - each element is either the string "imm" or "exp", denoting the type of the corresponding transition in the "transitions" property
        rate_transitions = [];  %[float array] - element i corresponds to the weight/rate of transition i
        input_arcs = [];        %[int matrix] - entry (i,j) corresponds to the input arc weight between place i and transition j. If 0, no arc exists
        output_arcs = [];       %[int matrix] - entry (i,j) corresponds to the output arc weight between transition i and place j. If 0, no arc exists
        arcs = struct();        %Struct that holds arcs auxiliary variables
        initial_marking = [];   %[int array] - each element corresponds to the number of tokens present in the corresponding place in the initial marking.
        current_marking = [];   %[int array] - representing current marking
        place_rewards = [];     %[float array] - each element corresponds to the place rewards for place with the same index in places property
        transition_rewards = [];%[float array] - each element corresponds to the transition rewards for the transition with the same index in the transitions property.
    end
    
    methods
        function GSPN = GSPNR()
            %Instantiating empty instance of GSPNR Model
            GSPN.arcs.places = [string.empty];
            GSPN.arcs.transitions = [string.empty];
            GSPN.arcs.types = [string.empty];
            GSPN.arcs.weights = [];
        end
        function add_places(GSPN, places, ntokens)
            %Adds places to the GSPNR object
            %Input:
            %   places [string array] - places to add
            %   ntokens [int array] - tokens in the added places
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
            %Returns the index in the obj.places array of the place_name input
            %Input:
            %   place_name [string]
            %Output:
            %   index [int]
            index = find(GSPN.places == place_name);
            if isempty(index)
                warning("Could not find place, returned 0");
                index = 0;
            end
        end
        function set_initial_marking(GSPN, new_marking)
            %Sets the initial marking of the GSPNR object
            %Input:
            %   new_marking [int array] - new initial marking, must be the same length as the obj.places property
            nPlaces = length(GSPN.places);
            if nPlaces ~= length(new_marking)
                error("New marking must be consistent with the number of places that exist")
            end
            GSPN.initial_marking = new_marking;
            GSPN.current_marking = GSPN.initial_marking;
        end
        function set_marking(GSPN, new_marking)
            %Sets the current marking to the GSPN object
            %Input
            %   new_marking [int array] - new marking, must be the same length as the obj.places property
            nPlaces = length(GSPN.places);
            if nPlaces ~= length(new_marking)
                error("New marking must be consistent with the number of places that exist")
            end
            GSPN.current_marking = new_marking;
        end
        function type = check_marking_type(GSPN)
            %Returns either "TAN", "DET" or "RAN" if the current marking is either tangible, deterministic (expecting policy action), or expecting a random switch, respectively.
            %Output:
            %   type [string] - either "TAN"/"DET"/"RAN";
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
           %Input:
           %    places [string array] - set of places to be removed
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
            %Adds immediate and/or exponential transitions to the GSPNR object
            %Input:
            %   transitions [string array] - name of transitions to add
            %   types       [string array] - type of transitions to add
            %   (with the same order as the transitions input) each element
            %   must be either "imm" or "exp"
            %   rates       [string array] - rates/weights of transitions
            %   to add, interpreted as weights when transition is immediate
            %   and as rates when transition is exponential
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
            %Returns the index of the transition given in the obj.transitions array
            %Input:
            %   trans_name [string] - name of transition
            index = find(GSPN.transitions == trans_name);
            if isempty(index)
                warning("Could not find transition, returned 0");
                index = 0;
            end
        end
        
        function remove_transitions(GSPN, transitions)
           %Removes transitions from the GSPNR object
           %Input:
           %    transitions [string array] - set of transitions to be removed
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
            %Returns list of exponential transitions and their corresponding indices
            exp_trans_indices = find(GSPN.type_transitions == "exp");
            nEXPTrans = size(exp_trans_indices, 2);
            for t_index = 1:nEXPTrans
                exp_trans(t_index) = GSPN.transitions(exp_trans_indices(t_index));
            end
        end
        
        function add_arcs(GSPN, places, transitions, type, weights)
            %Adds input and output arcs between existing places and transitions to the GSPNR object
            %Input:
            %   places      [string array] - set of places where arcs connect
            %   transitions [string array] - set of transitions where arcs connect
            %   type        [string array] - type for each arc, either
            %                                "in" or "out", corresponding to an arc from a place to a
            %                                transition, or from a transition to a place,
            %                                correspondingly
            %   weights     [int array]    - multiplicity of each arc
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
        
        function change_type_of_transition(GSPN, name, new_type)
            trans_index = GSPN.find_transition_index(name);
            if trans_index == 0
                error('Could not find transition');
            else
                GSPN.type_transitions(trans_index) = new_type;
            end
        end
        
        function change_rate_of_transition(GSPN, name, new_rate)
            %Changes the transition rate/weight for a given transition
            %Input:
            %   name [string] - name of the transition;
            %   new_rate [float/int] - new rate for transition
            trans_index = GSPN.find_transition_index(name);
            if trans_index == 0
                error('Could not find transition');
            else
                GSPN.rate_transitions(trans_index) = new_rate;
            end
        end
            
        
        function target_trans = find_target_trans(GSPN, place)
            %Finds all transitions connected to a given place by input arcs
            %Input:
            %   name [string] - name of place
            place_index = GSPN.find_place_index(place);
            arc_row_vector = GSPN.input_arcs(place_index, :);
            trans_indices = find(arc_row_vector);
            target_trans = translate_to_names(GSPN.transitions, trans_indices);
        end
        
        function fire_transition(GSPN, transition)
            %Fires the transition in the input and updates the current marking property
            %Input:
            %   transition [string] - name of transition to fire
           transition_index = find(strcmp(GSPN.transitions, transition));
           intermediate_marking = GSPN.current_marking - GSPN.input_arcs(:,transition_index)';
           if ( any(intermediate_marking<0) )
              error('Transition was not enabled and so cannot fire') 
           end
           final_marking = intermediate_marking + GSPN.output_arcs(transition_index,:);
           GSPN.current_marking = final_marking;
        end
        
        
        function format(GSPN,argument_list)
            %Finds all elements (places or transitions) that contain tags
            %of the form <n>, where n is a positive integer, and
            %substitutes the tag with nth element of the argument list.
            %Inputs:
            %   argument_list [string array] - list of arguments corresponding to each tag
            %Example:
            %   GSPNR has place "Travel_from_<1>to<2>", GSPNR.format(["L1"
            %   "L2"]) will convert place to name "Travel_from_L1to_L2".
            nArguments = size(argument_list, 2);
            for blank_index = 1:nArguments
                old_tag = "<"+string(blank_index)+">";
                GSPN.places = strrep(GSPN.places, old_tag, argument_list(blank_index));
                GSPN.transitions = strrep(GSPN.transitions, old_tag, argument_list(blank_index));
                GSPN.arcs.places = strrep(GSPN.arcs.places, old_tag, argument_list(blank_index));
                GSPN.arcs.transitions = strrep(GSPN.arcs.transitions, old_tag, argument_list(blank_index));
            end
        end
        
        function [policy_struct, error, nStates] = policy_synthesis(GSPN, timeout, discount_factor, wait)
            %Computes policy for GSPNR Model by converting to MDP and
            %running value iteration on the equivalent MDP
            %Input:
            %   timeout         [int]     - maximum duration that the value iteration
            %                               solver can run;
            %   discount_factor [float]   - value between 0 and 1;
            %   wait            [logical] - if true, runs on equivalent MDP
            %                               with wait states and wait
            %                               action;
            %Output:
            %   policy_struct   [struct]  - struct that holds the GSPNR that
            %                               the policy synthesis module was
            %                               run on, equivalent MDP, and
            %                               mapping between MDP states and
            %                               GSPNR marking
            %   error           [float]   - maximum error found in the last
            %                               iteration done by value iteration;
            %   nStates         [int]     - number of states found in the
            %                               equivalent MDP
            policy_struct = struct();
            disp("Started converting to MDP");
            if wait == true
                [mdp, markings, states, types] = GSPN.toMDP();
            else
                [mdp, markings, states, types] = GSPN.toMDP_without_wait();
            end
            disp("Finished converting to MDP");
            mdp.check_validity();
            mdp.set_enabled_actions();
            nStates = mdp.nStates;
            % Evaluating optimal policy
            disp("Starting value iteration");
            [values, policy, error] = value_iteration(mdp, 1, discount_factor, 0.01, timeout);
            policy_struct.gspn = GSPN;
            policy_struct.state_index_to_markings = markings;
            policy_struct.states = states;
            policy_struct.mdp = mdp;
            policy_struct.mdp_policy = policy;
        end
        
        function total_memory = measure_memory(GSPN)
            total_memory = 0;
            properties_list = string(properties(GSPN));
            nProperties = size(properties_list, 1);
            for p_index = 1:nProperties
                variable = GSPN.(properties_list(p_index));
                var_info = whos('variable');
                total_memory = total_memory + var_info.bytes;
            end
            %total memory in MB
            total_memory = total_memory/1024;
        end
    end
    
end


