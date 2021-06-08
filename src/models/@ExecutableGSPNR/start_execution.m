function start_execution(obj)
%ROSEXECUTIONMANAGER Summary of this function goes here
%
%   gspn            (ExecutableGSPNR object)    GSPNR to be executed;
%
%   RobotsList      (string list)               Name of robots in the system (must match
%                                               the number of tokens in the initial
%                                               marking)
%
%   DistRobots      (integer list)              (1xn) Robots integer list, sets the initial
%                                               distribution of robots throughout the
%                                               system. Is needed to map robots to tokens.
%                                               j = distRobots(i) means robot i starts off
%                                               in the place in the gspn with index j
%----------------------
    %Create buffer global variable that ResultFcn of action clients can
    %fill
    delete(timerfindall)
    global logID;
    global finished_actions_buffer;
    global finished_actions_count;
    finished_actions_count = 0;
    global lock;
    finished_actions_buffer = struct();
    lock = 0;
    cleanupObj = onCleanup(@CleaningAfterInterrupt);
    log_name = inputname(1)+string(datestr(now, 'dd_mm:HH:MM:SS'));
    log_path = "logs/execution/"+log_name+".txt";
    logID = fopen(char(log_path), 'w');
    log_firing(logID, obj, 0, [], [], []);
    %Checking GSPN properties
    fprintf('----------------------\nChecking GSPNR properties\n');
    obj.check_robot_ambiguity();
    obj.check_robot_conservation();
    %Connect to ROS Network
    fprintf('----------------------\nConnecting to ROS Network\n');
    rosinit
    disp(obj.launch_cmd)
    rosnode list
    [status, cmdout] = system(obj.launch_cmd)
    pause();
    rosnode list
    rosaction list
    %rosnode list
    %Check available actions
    actionlist = string(rosaction("list"));
    %----------------------
    %Check that all interface action servers for each robot are available
    %and start action clients for each one
    interface_action_clients = struct();
    for r_index = 1:obj.nRobots
        interface_action_server_name = obj.interface_action_servers(r_index);
        [client, msg] = rosactionclient(interface_action_server_name);
        client.ResultFcn = @FinishedAction;
        interface_action_clients(r_index).client = client;
        interface_action_clients(r_index).goalmsg = msg;
        waitForServer(interface_action_clients(r_index).client);
    end
    fprintf('----------------------\nAll interface action servers needed are correctly launched\n----------------------\n');
    %Check that all actions that are supposed to be run are actually available
    nActionPlaces = size(obj.place_actions, 2);
    for ap_index = 1:nActionPlaces
        if ~isempty(obj.place_actions(ap_index).place_name)
            for r_index = 1:obj.nRobots
                robot_name = obj.robot_list(r_index);
                action_server_name = "/"+robot_name + "/" + obj.place_actions(ap_index).server_name;
                if isempty(find(actionlist == action_server_name))
                    error_string = "Action server for action '"+action_server_name+"' is not running";
                    error(error_string);
                end
            end
        end
    end

    fprintf('----------------------\nAll robot action servers needed are correctly launched\n----------------------\n');
    %----------------------
%     %Checking that there is no ambiguity in GSPNR regarding tokens/robots
%     if exec.ambiguity == true
%         error("Before executing GSPNR, ambiguity must be resolved.")
%     end
    %Checking initial robot distribution and initializing variables
    nRobots = obj.nRobots;
    RobotPlaces = obj.robot_initial_locations
    %Initializing flag vector for all robots
    ExecutionFlags = repmat("FIN", [1 nRobots]);

    simple_transitions = obj.find_simple_exp_transitions();
    nSimpleTransitions = size(simple_transitions, 2);
    ExponentialFlags = repmat("FIN", [1 nSimpleTransitions]);

    wait_state = false;
    %Main firing loop
    done = false;

    while (~done)

        while (finished_actions_count ~= 0)
          fprintf("\n\nPROCESSING BUFFER MESSAGES---------------");
          %Processing all actions that returned in the meantime;
          fin_action = finished_actions_buffer(1);
          if fin_action.action == true
              %Processing exponential transition that represents the end of
              %an action
              place_done = obj.places(fin_action.place_index)
              transitions = obj.find_target_trans(obj.places(fin_action.place_index));
              if size(transitions, 2) ~= 1
                  error("Action places are only allowed to be connected by a single input arc to a single transition");
              end
              log_firing(logID, obj, 1, transitions(1), RobotPlaces, ExecutionFlags);
              ExecutionFlags(fin_action.robot_index) = "FIN";
              robots_involved = fin_action.robot_index;
              obj.fire_transition(transitions(1));
              wait_state = false;
              RobotPlaces = obj.update_robot_places(transitions(1), RobotPlaces, robots_involved);
              log_firing(logID, obj, 2, transitions(1), RobotPlaces, ExecutionFlags);
          else
              %Processing exponential transition that is not involved with
              %any robot places, can independently fire
              transition_index = fin_action.robot_index;
              transition_name = obj.transitions(transition_index);
              timer_name = fin_action.timer_name;
              [imm, exp] = obj.enabled_transitions();
              if ~isempty(find(exp == transition_name))
                  log_exponential_transitions(logID, obj, 0, transition_name, ExponentialFlags, [], timer_name);
                  obj.fire_transition(transition_name);
                  wait_state = false;
                  in_vector_index = find(simple_transitions == transition_name);
                  ExponentialFlags(in_vector_index) = "FIN";
                  log_exponential_transitions(logID, obj, 1, transition_name, ExponentialFlags, [], timer_name);
              end

          end
          done_cleaning_buffer = 0;
          while ~done_cleaning_buffer
              if lock == 0
                  lock = 1;
                  finished_actions_buffer(1) = [];
                  finished_actions_count = finished_actions_count - 1;
                  lock = 0;
                  done_cleaning_buffer = 1;
              end
          end
           fprintf("\n\nFINISHED BUFFER MESSAGES---------------");

        end

        marking_type = obj.check_marking_type();

        if ( marking_type == "TAN" || wait_state == true)
            %fprintf("\nCHECKING IF GOALS NEED TO BE SENT----------------------\n");
            for r_index = 1:obj.nRobots
                %Check that it is a new action
                flag = ExecutionFlags(r_index);
                if flag == "FIN"
                    robot_name = obj.robot_list(r_index)
                    place_index = RobotPlaces(r_index)
                    print = "Sent goal of place - "+obj.places(place_index)+"to robot "+robot_name
                    if ~isempty(obj.place_actions(place_index).place_name)
                        log_goals(logID, obj, 1, place_index, r_index, ExecutionFlags);
                        interface_action_clients(r_index).goalmsg.Order = int32(place_index);
                        ExecutionFlags(r_index) = "EXE";
                        sendGoal(interface_action_clients(r_index).client, interface_action_clients(r_index).goalmsg);
                        log_goals(logID, obj, 0, place_index, r_index, ExecutionFlags);
                    end
                end
            end
            for st_index = 1:nSimpleTransitions
                flag = ExponentialFlags(st_index);
                transition_name = obj.simple_exp_transitions(st_index);
                [imm, exp] = obj.enabled_transitions();
                if ~isempty(find(exp == transition_name))
                    exec_trans_index = obj.find_transition_index(transition_name);
                    transition_rate = obj.rate_transitions(exec_trans_index);
                    if flag == "FIN"
                       delay = round(exprnd(1/transition_rate), 2);
                       exp_timer = timer('StartDelay', delay);
                       exp_timer.TimerFcn = {@FinishedExponentialTransition, exec_trans_index};
                       start(exp_timer);
                       ExponentialFlags(st_index) = "EXE";
                       log_exponential_transitions(logID, obj, 2, transition_name, ExponentialFlags, delay, exp_timer.Name);
                    end
                else
                    continue
                end
            end

        else
        %Marking either "RAN" or "DET"
            transition = obj.get_policy(obj.current_marking);
            if transition == ""
                fprintf("No policy action for current marking");
                [imm, exp] = obj.enabled_transitions();
                nTransitions = size(imm, 2);
                rn_trans = randi(nTransitions);
                %fprintf("\nWill fire transition (random) - %s", imm(rn_trans));
                robots_involved = obj.check_robots_involved(imm(rn_trans), RobotPlaces);
                log_firing(logID, obj, 1, imm(rn_trans), RobotPlaces, ExecutionFlags);
                obj.fire_transition(imm(rn_trans));
                wait_state = false;
                RobotPlaces     = obj.update_robot_places(imm(rn_trans), RobotPlaces, robots_involved);
                log_firing(logID, obj, 2, imm(rn_trans), RobotPlaces, ExecutionFlags);
            elseif transition == "WAIT"
                fprintf("Policy action was WAIT")
                wait_state = true;
            else
                %fprintf("\nWill fire transition (policy) - %s", transition);
                fprintf("Will fire transition from policy");
                robots_involved = obj.check_robots_involved(transition, RobotPlaces);
                log_firing(logID, obj, 1, transition, RobotPlaces, ExecutionFlags);
                obj.fire_transition(transition);
                wait_state = false;
                RobotPlaces     = obj.update_robot_places(transition, RobotPlaces, robots_involved);
                log_firing(logID, obj, 2, transition, RobotPlaces, ExecutionFlags);
            end
        end
        pause(0.1);
    end

end

function FinishedAction(~, msg, s, ~)
    global finished_actions_buffer
    global finished_actions_count
    global lock
    fin_action = struct();
    fin_action.action = true;
    fin_action.robot_index = msg.Message.Sequence(2);
    fin_action.place_index = msg.Message.Sequence(1);
    done = 0;
    while ~done
        if lock == 0
            lock = 1;
            if finished_actions_count == 0
                finished_actions_buffer = fin_action;
                finished_actions_count = finished_actions_count + 1;
            else
                finished_actions_buffer = cat(2, finished_actions_buffer, fin_action);
                finished_actions_count = finished_actions_count + 1;
            end
            lock = 0;
            done = 1;
        end
    end
end

function FinishedExponentialTransition(obj,~,transition_index)
    global finished_actions_buffer
    global finished_actions_count
    global lock
    fin_action = struct();
    fin_action.action = false;
    fin_action.robot_index = transition_index;
    fin_action.place_index = 0;
    fin_action.timer_name = obj.Name;
    done = 0;
    while ~done
        if lock == 0
            lock = 1;
            if finished_actions_count == 0
                finished_actions_buffer = fin_action;
                finished_actions_count = finished_actions_count + 1;
            else
                finished_actions_buffer = cat(2, finished_actions_buffer, fin_action);
                finished_actions_count = finished_actions_count + 1;
            end
            lock = 0;
            done = 1;
        end
    end
end

function CleaningAfterInterrupt()
    global logID;
    fclose(logID);
    disp('Shutting down MATLAB ROS node')
    rosshutdown;
end
