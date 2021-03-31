clear
clc

adj = [0 , 1, 0 ;...
       1 , 0, 1 ;...
       0 , 1, 0];

map = digraph(adj, {'lab' 'office' 'hallway'},'omitselfloops');

plot(map)

action_dict = struct();

action_dict.lab = ['inspect'];
action_dict.hallway = ['inspect'];

[nGSPN, gspn_struct] = ImportfromGreatSPN('testExecutableGSPN.PNPRO');

%exec = ExecutableGSPNR(map, action_dict, gspn_struct);
