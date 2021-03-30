function ROSExecutionManager(gspn,RobotsList,distRobots, map_pl_act_msg)
%ROSEXECUTIONMANAGER Summary of this function goes here
%
%   gspn            (GSPNR object)  GSPNR to be executed;
%
%   nRobots         (integer)       Number of robots in the system (must match
%                                   the number of tokens in the initial
%                                   marking)
%
%   distRobots      (integer list)  (1xn) Robots integer list, sets the initial
%                                   distribution of robots throughout the
%                                   system. Is needed to map robots to tokens.
%                                   j = distRobots(i) means robot i starts off
%                                   in the place in the gspn with index j
%
%   map_pl_act_msg  (struct array)  Cell array of dimensions 1xK, where K
%                                   is the number of different robot types.
%                                   map_pl_act_msg{i} is a string matrix
%                                   (3xN) where each column corresponds to
%                                   a map between a place in the GSPNR and
%                                   an action type and a corresponding goal
%                                   message to be sent to the action server
%                                   E.g. map_pl_act_msg{1} = 
%                                   ['TravelL1L2'    ,
%                                   ['Travel'        ,
%                                   ['Travel/GoalMsg',




%----------------------
%Connect to ROS Network
rosinit
%Check available actions
actionlist = string(rosaction("list"));
%----------------------
%Check that all actions that are supposed to be run are actually available
available_robot_types = fieldnames(map_pl_act_msg);
nRobotTypes = size(available_robot_types,2);
for r_index = 1:nRobotTypes
    robot_type = available_robot_types{r_index};
    robot_struct = map_pl_act_msg.(robot_type);
    available_action_places = fieldnames(robot_struct);
    nActionPlaces = size(available_action_places,2);
    for a_index = 1:nActionPlaces
        action_place = robot_struct.(available_action_places{a_index});
        action_name = action_place.action_name;
        if isempty(find(actionlist == action_name))
            error_string = "Action server for action '"+action_name+"' is not running";
            error(error_string);
        end
    end
end
fprintf('----------------------\nAll action servers needed are correctly launched\n----------------------\n');
%----------------------
%Checking initial robot distribution and initializing variables
nRobots = size(2, RobotsList);
RobotsPlace = distRobots;

end

function [new_robot_places] = UpdateRobotPlaces(gspn, transition, old_robot_places)

end