function [emb_MDP, covered_marking_list, covered_state_list, covered_state_type] = toMDP(GSPN)
   %Transforms the GSPNR object into the equivalent MDP
   %Output:
   %    emb_MDP                 [MDP object] - instance of MDP class;
   %    covered_marking_list    [int matrix] - each row of this matrix
   %                                           represents a marking that was 
   %                                           found by the algorithm;
   %    covered_state_list      [string array] - each element is a state
   %                                             name in the MDP Model,
   %                                             equivalent to the marking
   %                                             in the same position in
   %                                             the covered_marking_list
   %                                             output
   %    covered_state_type      [string array] - each element contains a
   %                                             string defining the type
   %                                             of state in the MDP
   
   
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
              %RANDOM SWITCH
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
   
   disp("Finished creating all states - continuing to uniformization");
   emb_MDP.consolidation_uniformization(covered_state_type);
   disp("Finished uniformization - continuing to add rewards");

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
      state_reward = dot(GSPN.place_rewards,marked_places~=0);
      if state_type == "TAN"
        normalized_reward = state_reward/emb_MDP.eta;
        emb_MDP.set_reward(state_name, "EXP", normalized_reward);
      end
   end
           
end
