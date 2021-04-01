clc
clear

[nGSPN gspn_list] = ImportfromGreatSPN("merging_test.PNPRO");

gspn2 = gspn_list.GSPN2;

%gspn2.remove_transitions(["t1"])

merged = MergeGSPNR(gspn_list.GSPN1, gspn_list.GSPN2)