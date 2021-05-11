function log_firing(fileID, execGSPNR, firing, transition, robot_places, robot_flags)
%LOG_TO_FILE Summary of this function goes here
%   No transition firing involved - firing = 0 - print initial states
%   Before firing transition - firing = 1
%   After firing transition - firing = 2
    if firing == 0
        %Just log initial state of important variables and log time that
        %execution starts
        fprintf(fileID, 'Starting execution at %s\n', datestr(now, 'HH:MM:SS'));
        fprintf(fileID, 'Places -> ');
        fprintf(fileID, mat2str(execGSPNR.places));
        fprintf(fileID, '\nRobots -> ');
        fprintf(fileID, mat2str(execGSPNR.robot_list));
        fprintf(fileID, '\nCurrent Marking -> ');
        fprintf(fileID, mat2str(execGSPNR.current_marking));
        fprintf(fileID, '\n Transitions -> ');
        fprintf(fileID, mat2str(execGSPNR.transitions));
        fprintf(fileID, '\n%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n\n');
    elseif firing == 1
        %State of the robots before firing transition
        fprintf(fileID, '\nBefore firing transition %s, at time %s the state of the system was the following:', transition, datestr(now, 'HH:MM:SS'));
        fprintf(fileID, '\nRobots Places -> ');
        fprintf(fileID, mat2str(robot_places));
        fprintf(fileID, '\nRobot Flags -> ');
        fprintf(fileID, mat2str(robot_flags));
        fprintf(fileID, '\nCurrent Marking -> ');
        fprintf(fileID, mat2str(execGSPNR.current_marking));
        fprintf(fileID, '\n------------------------------------------------');
    elseif firing == 2
        %State of the robots after firing transition
        fprintf(fileID, '\nAfter firing transition %s the state of the system was the following:', transition);
        fprintf(fileID, '\nRobots Places -> ');
        fprintf(fileID, mat2str(robot_places));
        fprintf(fileID, '\nRobot Flags -> ');
        fprintf(fileID, mat2str(robot_flags));
        fprintf(fileID, '\nCurrent Marking -> ');
        fprintf(fileID, mat2str(execGSPNR.current_marking));
        fprintf(fileID, '\n%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n\n');
    end 
end

