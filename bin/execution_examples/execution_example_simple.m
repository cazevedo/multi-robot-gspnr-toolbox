clc
clear

PNPRO_path = 'execution.PNPRO';
pnml_file = 'execution_example.xml';
yaml_file = 'example.yaml';
yaml_filepath = 'test.yaml';

gspn = ImportfromPIPE(pnml_file);
%PrepareYAML(gspn, yaml_filepath);
%[nGSPN, gspn_list] = ImportfromGreatSPN(PNPRO_path);
action_place_struct = PrepareROSExecution(gspn, yaml_file);

exec = ExecutableGSPNR();
exec.import_nonExecutable(gspn, action_place_struct);
%ROSExecutionManager(gspn, [], [], action_place_struct);



              