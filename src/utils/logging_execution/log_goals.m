function log_goals(fileID, execGSPNR, before, place_index, robot_index, ExecutionFlags)
%LOG_TO_FILE Summary of this function goes here
%   Before firing simple exponential transition - firing = 0
%   After firing simple expoenetial transition - firing = 1
%   Setting exponential transitio timer - firing = 2
%   Sending goals to robots - firing = 3
    if before
        fprintf(fileID, '\nSending goal of place index -> %s, to robot with index -> %s, at time %s', string(place_index), string(robot_index), datestr(now, 'HH:MM:SS'));
        fprintf(fileID, '\nExecution flags before sending the goal: %s', mat2str(ExecutionFlags));
        fprintf(fileID, '\n------------------------------------------------');
    else
        fprintf(fileID, '\nSent goal at time %s', datestr(now, 'HH:MM:SS'));
        fprintf(fileID, '\nExecution flags after sending goal: %s', mat2str(ExecutionFlags));
        fprintf(fileID, '\n%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n\n');
    end 
end

