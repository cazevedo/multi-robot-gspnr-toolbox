function log_exponential_transitions(fileID, execGSPNR, firing, transition, exponential_flags, delay, timer_name)
%LOG_TO_FILE Summary of this function goes here
%   Before firing simple exponential transition - firing = 0
%   After firing simple expoenetial transition - firing = 1
%   Setting exponential transitio timer - firing = 2
%   Sending goals to robots - firing = 3
    if firing == 0
        %State of the robots before firing transition
        fprintf(fileID, '\nBefore firing simple exponential transition %s, at time %s the state of the system was the following:\n', transition, datestr(now, 'HH:MM:SS'));
        fprintf(fileID, '\nCurrent Marking -> ');
        fprintf(fileID, mat2str(execGSPNR.current_marking));
        fprintf(fileID, '\n Exponential flags -> ');
        fprintf(fileID, mat2str(exponential_flags));
        fprintf(fileID, '\n Timer name -> %s', timer_name);
        fprintf(fileID, '\n------------------------------------------------');
    elseif firing == 1
        %State of the robots after firing transition
        fprintf(fileID, '\nAfter firing simple exponential transition %s, at time %s the state of the system was the following:\n', transition, datestr(now, 'HH:MM:SS'));
        fprintf(fileID, '\nCurrent Marking -> ');
        fprintf(fileID, mat2str(execGSPNR.current_marking));
        fprintf(fileID, '\n Exponential flags -> ');
        fprintf(fileID, mat2str(exponential_flags));
        fprintf(fileID, '\n Timer name -> %s', timer_name);
        fprintf(fileID, '\n%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n\n');
    elseif firing == 2
        %Log delay value for particular transition
        fprintf(fileID, '\nThe simple exponential transition %s started a timer with %s seconds, at time %s', transition, string(delay), datestr(now, 'HH:MM:SS'));
        fprintf(fileID, '\n Exponential flags -> ');
        fprintf(fileID, mat2str(exponential_flags));
        fprintf(fileID, '\n Timer name -> %s', timer_name);
        fprintf(fileID, '\n%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n\n');
        
        
        
    end 
end

