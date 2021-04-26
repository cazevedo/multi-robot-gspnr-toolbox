# Instructions to run "meeting_example.m"

1. Clone the repo:
    ~~~
    git clone https://github.com/cazevedo/multi-robot-gspnr-toolbox.git
    ~~~
2. Clone the ROS Package that implements the action servers used in the example into your catkin workspace:
    ~~~
    roscd
    git clone https://github.com/cazevedo/multi_robot_home_clean.git
    catkin build --this
    ~~~
3. In a different terminal, start up your roscore:
    ~~~
    roscore
    ~~~
3. Start up the dummy action servers for the Navigate/Mop/Vacuum actions by:
    ~~~
    roslaunch multi_robot_home_clean launch_action_servers.launch 
    ~~~
4. In MATLAB, add to path the toolbox, and enter into the main directory of the toolbox;
5. Run "meeting_example.m"
