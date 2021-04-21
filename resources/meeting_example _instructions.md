# Instructions to run "meeting_example.m"

1. Add “multi-robot-gspnr-toolbox” and subfolders to Matlab Path;
2. Place the provided catkin package “matlab_execution_tests” in your catkin workspace and build using the command:
    ~~~
    catkin_make
    ~~~
3. Start up the dummy action servers for the Navigate/Mop/Vacuum actions either by:
     Executing each one individually by running the following commands (each one in its own terminal window:
    ~~~
    rosrun matlab_execution_tests navigating_server_tb1.py
    rosrun matlab_execution_tests mopping_server_tb1.py
    rosrun matlab_execution_tests vacuuming_server_tb1.py
    rosrun matlab_execution_tests navigating_server_tb2.py
    rosrun matlab_execution_tests mopping_server_tb2.py
    rosrun matlab_execution_tests vacuuming_server_tb2.py
    ~~~

    Using the launch file (NOT TESTED) by running the following command:
    ~~~
    roslaunch matlab_execution_tests launch_action_servers.launch
    ~~~

4. Change variable “catkin_package_path” to a path of a valid catkin package – this is where the toolbox’s interface action servers will be saved and executed out of;
5. Start a roscore;
6. Run “meeting_example.m”;
7. When prompted, run the python interface scripts. The name of each of these scripts is:
    ~~~
    matlab_interface_server_<robot_name>.py
    ~~~
    By default, as the robots names’ are “tb1” and “tb2”, and you must run the commands:
    ~~~
    rosrun <catkin_package_name> matlab_interface_server_tb1.py
    rosrun <catkin_package_name> matlab_interface_server_tb2.py
    ~~~
