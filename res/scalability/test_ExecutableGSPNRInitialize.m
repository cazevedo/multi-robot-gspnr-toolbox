function [GSPNRModel, parameters] = test_ExecutableGSPNRInitialize(nLocations)
    
    parameters = struct();
    %Importing GSPNR Model from previous run test
    mat_filename = "locations"+string(nLocations)+"_creation_test.mat";
    load(mat_filename, 'GSPNR');
    
    executableModel.initialize(GSPNR, YAML_file, []);
    
end

