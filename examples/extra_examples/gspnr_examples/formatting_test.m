clc
clear

[nGSPN gspn_list] = ImportfromGreatSPN("formatting_test.PNPRO");

gspn_list.GSPN.format(["hallway", "lab"])

