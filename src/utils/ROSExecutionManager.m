function ROSExecutionManager(exec,DistRobots)
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
    global finished_actions_buffer;
    global finished_actions_count;
    finished_actions_count = 0;
    global lock;
    finished_actions_buffer = struct();
    lock = 0;
    %Connect to ROS Network
    fprintf('----------------------\nConnecting to ROS Network\n');
    %rosinit
    %Check available actions
    actionlist = string(rosaction("list"));
    %----------------------
    %Check that all interface action servers for each robot are available
    %and start action clients for each one
    interface_action_clients = struct();
    for r_index = 1:exec.nRobots
        interface_action_server_name = exec.interface_action_servers(r_index);
%         if isempty(find(actionlist == interface_action_server_name))
%             error_string = "Interface action server for robot '"+exec.robot_list(r_index)+"' is not running";
%             error(error_string);
%         end
        [client, msg] = rosactionclient(interface_action_server_name);
        client.ResultFcn = @FinishedAction;
        interface_action_clients(r_index).client = client;
        interface_action_clients(r_index).goalmsg = msg;
        waitForServer(interface_action_clients(r_index).client);
    end
    fprintf('----------------------\nAll interface action servers needed are correctly launched\n----------------------\n');
    %Check that all actions that are supposed to be run are actually available
    nActionPlaces = size(exec.place_actions, 2);
    for ap_index = 1:nActionPlaces
        if ~isempty(exec.place_actions(ap_index).place_name)
            for r_index = 1:exec.nRobots
                robot_name = exec.robot_list(r_index);
                action_server_name = "/"+robot_name + "_" + exec.place_actions(ap_index).server_name;
                if isempty(find(actionlist == action_server_name))
                    error_string = "Action server for action '"+action_server_name+"' is not running";
                    error(error_string);
                end
            end
        end
    end

    fprintf('----------------------\nAll robot action servers needed are correctly launched\n----------------------\n');
    %----------------------
    %Checking that there is no ambiguity in GSPNR regarding tokens/robots
    if exec.ambiguity == true
        error("Before executing GSPNR, ambiguity must be resolved.")
    end
    %Checking initial robot distribution and initializing variables
    nRobots = exec.nRobots;
    RobotPlaces = DistRobots;
    %Initializing flag vector for all robots
    ExecutionFlags = repmat("FIN", [1 nRobots]);

%     %Testing block for UpdateRobotPlaces
%     [imm, exp] = exec.enabled_transitions()
%     
%     RobotPlaces = UpdateRobotPlaces(exec, imm(1), RobotsPlace, [1])
%     exec.fire_transition(imm(1));
%     
%     [imm, exp] = exec.enabled_transitions()
%     
%     RobotsPlaces = UpdateRobotPlaces(exec, exp(1), RobotsPlace, [1])

    %Main firing loop
    done = false;

    while (~done)
        fprintf("\n\n");
        for r_index = 1:exec.nRobots
            robot_name = exec.robot_list(r_index);
            robot_flag = ExecutionFlags(r_index);
            robot_place = exec.places(RobotPlaces(r_index));
            fprintf("\nRobot %s has flag %s and is in place %s", robot_name, robot_flag, robot_place);
        end
        
        while (finished_actions_count ~= 0)
          %Processing all actions that returned in the meantime;
          fin_action = finished_actions_buffer(1);
          ExecutionFlags(fin_action.robot_index) = "FIN";
          place_done = exec.places(fin_action.place_index);
          transitions = exec.find_target_trans(exec.places(fin_action.place_index));
          if size(transitions, 2) ~= 1
              error("Action places are only allowed to be connected by a single input arc to a single transition");
          end
          fprintf("\nWill fire transition - %s", transitions(1));
          exec.places
          exec.current_marking
          RobotPlaces
          exec.fire_transition(transitions(1));
          robots_involved = CheckRobotsInvolved(exec, transitions(1), RobotPlaces);
          RobotPlaces = UpdateRobotPlaces(exec, transitions(1), RobotPlaces, robots_involved);
          done_cleaning_buffer = 0;
          exec.places
          exec.current_marking
          RobotPlaces
          while ~done_cleaning_buffer
              if lock == 0
                  lock = 1;
                  finished_actions_buffer(1) = [];
                  finished_actions_count = finished_actions_count - 1;
                  lock = 0;
                  done_cleaning_buffer = 1;
              end
          end
        end
        
        marking_type = exec.check_marking_type();
        
        if ( marking_type == "TAN" )
            for r_index = 1:exec.nRobots
                %Check that it is a new action
                flag = ExecutionFlags(r_index);
                if flag == "FIN"
                    robot_name = exec.robot_list(r_index);
                    place_index = RobotPlaces(r_index);
                    interface_action_clients(r_index).goalmsg.Order = int32(place_index);
                    ExecutionFlags(r_index) = "EXE";
                    sendGoal(interface_action_clients(r_index).client, interface_action_clients(r_index).goalmsg);
                    disp = "Sent goal of place index - "+string(interface_action_clients(r_index).goalmsg.Order)
                end
            end
        else
        %Marking either "RAN" or "DET"
            transition = exec.get_policy(exec.current_marking);
            if transition == ""
                %No policy action for current marking
                [imm, exp] = exec.enabled_transitions();
                nTransitions = size(imm, 2);
                rn_trans = randi(nTransitions);
                fprintf("\nWill fire transition (policy) - %s", imm(rn_trans));
                exec.fire_transition(imm(rn_trans));
                robots_involved = CheckRobotsInvolved(exec, imm(rn_trans), RobotPlaces);
                RobotPlaces     = UpdateRobotPlaces(exec, imm(rn_trans), RobotPlaces, robots_involved);
            else
                fprintf("\nWill fire transition (random) - %s", transition);
                exec.fire_transition(transition);
                robots_involved = CheckRobotsInvolved(exec, transition, RobotPlaces);
                RobotPlaces     = UpdateRobotPlaces(exec, transition, RobotPlaces, robots_involved);
            end
        end
        pause(1);
    end

end

function FinishedAction(~, msg, s, ~)
    global finished_actions_buffer
    global finished_actions_count
    global lock
    fin_action = struct();
    fin_action.robot_index = msg.Message.Sequence(2);
    fin_action.place_index = msg.Message.Sequence(1);
    done = 0;
    while ~done
        finished_actions_count;
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