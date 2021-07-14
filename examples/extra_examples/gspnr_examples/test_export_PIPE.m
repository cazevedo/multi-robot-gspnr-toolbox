clc
clear

[n, GSPN_struct] = ImportfromGreatSPN("test_toMDP_without_WAITS.PNPRO");

test = GSPN_struct.GSPN;
%%

ExportToPIPE(test, "test_function.xml", [1000 1000]);
    
    
    