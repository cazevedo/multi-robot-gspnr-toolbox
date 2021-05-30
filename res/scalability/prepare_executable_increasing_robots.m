clear
clc

max_nRobots = 6;
nLocations = 4;

%Parse all MAT files;
for nRobots = 1:max_nRobots
    mat_filename = "robots"+string(nRobots)+"_creation_test.mat";
    load(mat_filename, 'GSPNR');
    action_place = struct();
    nPlaces = size(GSPNR.places, 2);
    for p_index = 1:nPlaces
        place_name = GSPNR.places(p_index);
        if startsWith(place_name, "Mopping_")
            action_place.(place_name).server_name = 'MopActionServer';
            action_place.(place_name).action_name = 'MoppingAction';
            action_place.(place_name).package_name = 'multi_robot_home_clean';
            action_place.(place_name).message = struct("duration", 10);
        end
        if startsWith(place_name, "Navigating_")
            action_place.(place_name).server_name = 'NavigateActionServer';
            action_place.(place_name).action_name = 'NavigationAction';
            action_place.(place_name).package_name = 'multi_robot_home_clean';
            action_place.(place_name).message = struct("destination", 1);
        end
        if startsWith(place_name, "Vacuuming_")
            action_place.(place_name).server_name = 'VacuumActionServer';
            action_place.(place_name).action_name = 'VacuumingAction';
            action_place.(place_name).package_name = 'multi_robot_home_clean';
            action_place.(place_name).message = struct("duration", 10);
        end
    end
    save(mat_filename, 'action_place', '-append')
end
