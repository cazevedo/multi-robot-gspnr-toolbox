#!/bin/bash
if [ "$#" -ne 2 ]; then
    echo "Illegal number of parameters"
    echo "Usage : bash run_scalability_tests.sh 3 cpu_mem_usage.txt"
    exit
fi

# assign first received argument to variable PKG_NAME
MAX_NLOCATIONS=${1}
LOG_PATH=${2}
echo GROUP PID
echo $$

for NLOCATIONS in `seq 1 $MAX_NLOCATIONS`;
    do
      STRING1="run_ExecutableGSPNRExecution("
      STRING2=")"
      # echo $COMMAND

      COMMAND="matlab -nodisplay -nosplash -r $STRING1$NLOCATIONS$STRING2"
      $COMMAND &
      MATLAB_PID=$!
      echo EXITED MATLAB
      echo $MATLAB_PID
      source env/bin/activate
      python3 measure_cpu.py $MATLAB_PID $NLOCATIONS >> $LOG_PATH &
      sleep 5m
      kill -SIGINT $MATLAB_PID
      echo SENT SIGINT SIGNAL
      deactivate
      echo DONE TESTING FOR $NLOCATIONS
    done
