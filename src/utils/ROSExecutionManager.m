function [outputArg1,outputArg2] = ROSExecutionManager(gspn,nRobots,distRobots, map_pl_act_msg)
%ROSEXECUTIONMANAGER Summary of this function goes here
%
%   gspn            (GSPNR object)  GSPNR to be executed;
%
%   nRobots         (integer)       Number of robots in the system (must match
%                                   the number of tokens in the initial
%                                   marking)
%
%   distRobots      (integer list)  1xnRobots integer list, sets the initial
%                                   distribution of robots throughout the
%                                   system. Is needed to map robots to tokens.
%                                   j = distRobots(i) means robot i starts off
%                                   in the place in the gspn with index j
%
%   map_pl_act_msg  (cell array)    Cell array of dimensions 1xK, where K
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
%rosinit
%Check available actions
%actionlist = rosaction("list");
%----------------------

%

end