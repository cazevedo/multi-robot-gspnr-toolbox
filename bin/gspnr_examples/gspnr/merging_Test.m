clc
clear
% test GSPNS
[nGSPN, gspn_list] = ImportfromGreatSPN('merging_test.PNPRO');

% pre-conditions


%% Test 1 - Merging two empty GSPNRs

empty = MergeGSPNR(gspn_list.empty1, gspn_list.empty2);

%% Test 2 - Merging two GSPNRs with no common places

no_place = MergeGSPNR(gspn_list.no_common_places1, gspn_list.no_common_places2);

%% Test 3 - Merging two GSPNRs with no common transitions

no_trans = MergeGSPNR(gspn_list.no_common_trans1, gspn_list.no_common_trans2);

%% Test 4 - Merging two GSPNRs with no conflicts

no_conflict = MergeGSPNR(gspn_list.no_conflicts1, gspn_list.no_conflicts2);

%% Test 5 - Merging two GSPNRs that are exactly the same

same = MergeGSPNR(gspn_list.exact_same1, gspn_list.exact_same2);

%% Test 6 - Merging two regular GSPNRs

normal = MergeGSPNR(gspn_list.normal_test1, gspn_list.normal_test2);