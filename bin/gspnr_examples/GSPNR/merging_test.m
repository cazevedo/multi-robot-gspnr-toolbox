clc
clear

[nGSPN gspn_list] = ImportfromGreatSPN("merging_test.PNPRO");

MergeGSPNR(gspn_list.GSPN1, gspn_list.GSPN2)