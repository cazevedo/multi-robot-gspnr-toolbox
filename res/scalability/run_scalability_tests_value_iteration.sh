#!/bin/bash
if [ "$#" -ne 2 ]; then
    echo "Illegal number of parameters"
    echo "Usage : bash run_scalability_tests.sh 3 cpu_mem_usage.txt"
    exit
fi

# assign first received argument to variable PKG_NAME
MAX_NLOCATIONS=${1}
LOG_PATH=${2}

for NLOCATIONS in `seq 1 $MAX_NLOCATIONS`;
    do
      for TEST in `seq 1 3`
        do
          date
      	  STRING1="run_value_iteration("
          STRING2=")"
          # echo $COMMAND

          COMMAND="matlab -nodisplay -nosplash -batch $STRING1$NLOCATIONS$STRING2"
          $COMMAND &
          MATLAB_PID=$!
          # echo $MATLAB_PID
          python3 measure_cpu.py $MATLAB_PID $NLOCATIONS >> $LOG_PATH
          echo DONE $TEST FOR $NLOCATIONS
	done
        echo DONE TESTING FOR $NLOCATIONS
    done
