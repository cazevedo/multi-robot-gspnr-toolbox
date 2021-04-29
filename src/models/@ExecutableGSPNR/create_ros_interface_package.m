function create_ros_interface_package(obj)
    [status, toolbox_dir] = system("pwd");
    bash_cmd = "source " + strtrim(toolbox_dir) + "/res/create_ros_pkg.sh temp_matlab_gspnr_python_interface";
    system(bash_cmd);
    [status, catkin_ws] = system("echo $CMAKE_PREFIX_PATH");
    catkin_ws = strtrim(catkin_ws);
    catkin_ws = strsplit(catkin_ws, ":");
    catkin_ws = strrep(catkin_ws(1), "/devel","/src");
    package_dir = catkin_ws + "/temp_matlab_gspnr_python_interface";
    launch_temp_interface = obj.create_python_interface_scripts(package_dir);
    bash_cmd = "cd "+package_dir+" && "+"catkin build --this";
    system(bash_cmd);
    bash_cmd = "roslaunch temp_matlab_gspnr_python_interface "+launch_temp_interface+" &"
    [status, cmdout] = system(bash_cmd);
end