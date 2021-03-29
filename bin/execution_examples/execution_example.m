clc
clear

addpath('/home/antonio/Repos/multi-robot-gspnr-toolbox/src/models/');
addpath('/home/antonio/Repos/multi-robot-gspnr-toolbox/src/utils/');
addpath('/home/antonio/Repos/multi-robot-gspnr-toolbox/bin/execution_examples/')

PNPRO_path = 'execution.PNPRO';
pnml_file = 'execution_example.xml';
yaml_filepath = 'test.yaml';
yaml_file = 'example.yaml';

gspn = ImportfromPIPE(pnml_file);
PrepareYAML(gspn, yaml_filepath);
%[nGSPN, gspn_list] = ImportfromGreatSPN(PNPRO_path);
%PrepareROSExecution(gspn_list{1}, yaml_file);


              