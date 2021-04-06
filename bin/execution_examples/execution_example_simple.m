clc
clear

PNPRO_path = 'execution.PNPRO';
pnml_file = 'execution_example.xml';
yaml_file = 'example.yaml';
yaml_filepath = 'test.yaml';

gspn = ImportfromPIPE(pnml_file);
%PrepareYAML(gspn, yaml_filepath);
%[nGSPN, gspn_list] = ImportfromGreatSPN(PNPRO_path);
action_place_struct = PrepareROSExecution(yaml_file);

exec = ExecutableGSPNR();
exec.import_nonExecutable(gspn, action_place_struct);
exec.check_robot_ambiguity();
exec.check_marking_type()

marked_place_index = find(exec.initial_marking);

ROSExecutionManager(exec, ["turtlebot1"], [marked_place_index]);



              