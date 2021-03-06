#!/bin/bash
if [ "$#" -ne 3 ]; then
    echo "Illegal number of parameters"
    echo "Usage : bash run_scalability_tests.sh 3 cpu_mem_usage_matlab.txt cpu_mem_usage_ros.txt"
    exit
fi

source /opt/ros/melodic/setup.bash 
source /home/harode/tiago_ws/devel/setup.bash 

# assign first received argument to variable PKG_NAME
MAX_NLOCATIONS=${1}
MATLAB_LOG_PATH=${2}
ROS_LOG_PATH=${3}

echo GROUP PID
echo $$

for NLOCATIONS in `seq 1 $MAX_NLOCATIONS`;
    do
      for TEST in `seq 1 10`
	do
      	  STRING1="run_ExecutableGSPNRExecution("
          STRING2=")"
          # echo $COMMAND
          export LD_PRELOAD="/usr/lib/x86_64-linux-gnu/libcurl.so.4"
          COMMAND="matlab -nodisplay -nosplash -r $STRING1$NLOCATIONS$STRING2"
          $COMMAND &
          MATLAB_PID=$!
          echo $MATLAB_PID
          sleep 10s
          roslaunch temp_matlab_gspnr_python_interface matlab_interface_servers.launch &
          ROSLAUNCH_PID=$! 
          python3 measure_cpu.py $MATLAB_PID $NLOCATIONS >> $MATLAB_LOG_PATH &
          python3 measure_cpu.py $ROSLAUNCH_PID $NLOCATIONS >> $ROS_LOG_PATH &
          sleep 150s
          echo Finished sleeping
          kill -SIGINT $MATLAB_PID
          echo SENT SIGINT SIGNAL
 	done
      echo DONE TESTING FOR $NLOCATIONS
    done
