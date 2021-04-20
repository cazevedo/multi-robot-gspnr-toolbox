clc
clear

PNPRO_path = 'execution.PNPRO';
pnml_file = 'execution_example.xml';
yaml_file = 'example.yaml';
yaml_filepath = 'test.yaml';
src_matlab_interface = "catkin_ws/src/actionlib_experiments";

gspn = ImportfromPIPE(pnml_file);
%PrepareYAML(gspn, yaml_filepath);
%[nGSPN, gspn_list] = ImportfromGreatSPN(PNPRO_path);
action_place_struct = ReadfromYAML(yaml_file);

exec = ExecutableGSPNR();
exec.import_nonExecutable(gspn, action_place_struct);
nPlaces = size(exec.places, 2);
exec.set_all_places_as_robot_places();
%[mdp, markings, states, types] = exec.toMDP();
exec.check_robot_ambiguity();
exec.check_robot_conservation();
exec.check_marking_type();

in_office = zeros(1, nPlaces);
both_in_office(4) = 2;
in_hallway = zeros(1, nPlaces);
both_in_hallway(1) = 2;
in_lab = zeros(1, nPlaces);
both_in_lab(3) = 1;
pol_transitions = ["start_travel_OtoH"; "started_travel_HtoL"; "started_inspect"];
exec.set_policy([in_office;in_hallway;in_lab], pol_transitions);

exec.add_robots(["turtlebot1", "turtlebot2"]);
exec.create_python_interface_scripts(src_matlab_interface);

marked_place_index = find(exec.initial_marking);
exec.initial_marking

% [imm, exp] = exec.enabled_transitions()

% exec.fire_transition(imm(1))
% RobotPlaces = [marked_place_index, marked_place_index]
% robots_involved = CheckRobotsInvolved(exec, imm(1), RobotPlaces)
% RobotPlaces = UpdateRobotPlaces(exec, imm(1), RobotPlaces, robots_involved)
% exec.current_marking
% [imm, exp] = exec.enabled_transitions()
% exec.fire_transition(imm(1))
% robots_involved = CheckRobotsInvolved(exec, imm(1), RobotPlaces)
% RobotPlaces = UpdateRobotPlaces(exec, imm(1), RobotPlaces, robots_involved)
% exec.current_marking
% [imm, exp] = exec.enabled_transitions()
% robots_involved = CheckRobotsInvolved(exec, exp(1), RobotPlaces)
% exec.fire_transition(exp(1))
% RobotPlaces = UpdateRobotPlaces(exec, exp(1), RobotPlaces, robots_involved)
%exec.get_policy(exec.initial_marking)

ROSExecutionManager(exec, [marked_place_index, marked_place_index]);



              