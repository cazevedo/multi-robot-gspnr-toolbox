clc
clear

[n, GSPN_struct] = ImportfromGreatSPN("test_toMDP_without_WAITS.PNPRO");

base = readstruct("test_toMDP_without_WAITS.PNPRO", 'Filetype', 'xml');

test = GSPN_struct.GSPN;

GreatSPN_size = [50 50];

ExportToGreatSPN(test, "test_function.PNPRO", GreatSPN_size);