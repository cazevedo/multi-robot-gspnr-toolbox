clc
clear

addpath('/home/antonio/Repos/multi-robot-gspnr-toolbox/src/models/');
addpath('/home/antonio/Repos/multi-robot-gspnr-toolbox/src/utils/');
addpath('/home/antonio/Repos/multi-robot-gspnr-toolbox/bin/execution_examples/')

PNPRO_path = 'execution.PNPRO';
pnml_file = 'execution_example.xml';
yaml_file = 'example.yaml';

%gspn = ImportfromPIPE(pnml_file);
[nGSPN, gspn_list] = ImportfromGreatSPN(PNPRO_path);
%PrepareROSExecution(gspn, yaml_file);


              