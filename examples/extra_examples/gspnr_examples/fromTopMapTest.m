clear
clc

adj = [0 , 1, 0, ;...
       1 , 0, 1, ;...
       0 , 1, 0, ];

map = digraph(adj, {'kitchen' 'hallway' 'office'},'omitselfloops');

plot(map)

action_dict = struct();

action_dict.kitchen = ["cooking", "vacuuming"];
action_dict.hallway = ["vacuuming", "get_mail"];
action_dict.office  = ["vacuuming"];

[nGSPN, models] = ImportfromGreatSPN('testExecutableGSPN.PNPRO');

exec = GSPNRCreationfromTopMap(map, action_dict, models);

