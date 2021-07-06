clc
clear

%% Simulation Results

load("model_with_real_rates.mat", "homeModel");
load("simulation_results_real_rates.mat", "results");

chargingB0_trans_indices = find(results.transitions == "Charge_BS_B0");
chargedB0_trans_indices = find(results.transitions == "Charged_BS_B0");
chargingB1_trans_indices = find(results.transitions == "Charge_BS_B1");
chargedB1_trans_indices = find(results.transitions == "Charged_BS_B1");

chargingB0_timestamps = results.timestamps(chargingB0_trans_indices);
chargedB0_timestamps = results.timestamps(chargedB0_trans_indices);
chargingB1_timestamps = results.timestamps(chargingB1_trans_indices);
chargedB1_timestamps = results.timestamps(chargedB1_trans_indices);

chargingB0_timestamps = chargingB0_timestamps(1:size(chargedB0_timestamps, 1));
chargingB1_timestamps = chargingB1_timestamps(1:size(chargedB1_timestamps, 1));

simulation_chargingB0_durations = seconds(chargedB0_timestamps - chargingB0_timestamps);
simulation_chargingB1_durations = seconds(chargedB1_timestamps - chargingB1_timestamps);

vacuumed_all_trans_indices = find(results.transitions == "VacuumedAll");
nVacuumedAll = size(vacuumed_all_trans_indices, 1);
vacuumed_all_time = zeros(nVacuumedAll, 1);

charging_timestamps = results.timestamps(chargingB0_trans_indices);

nTransitions_fired = size(results.transitions, 1);
running_time = seconds(results.timestamps(nTransitions_fired));
average_reward_simulated = results.reward/running_time;

previous_time = 0;

for index = 1:nVacuumedAll
    index_in_results = vacuumed_all_trans_indices(index);
    time = results.timestamps(index_in_results);
    interval = time - previous_time;
    previous_time = time;
    vacuumed_all_time(index) = seconds(interval);
end

vacuuming_locations = ["L2", "L3", "L4", "L5", "L6"];
nVacuumingLocations = size(vacuuming_locations, 2);

for loc_index = 1:nVacuumingLocations
    location = vacuuming_locations(loc_index);
    start_string = "Vacuum_"+location;
    vacuum_location = find(startsWith(results.transitions, start_string));
    nVacuumLocation = size(vacuum_location, 1);
    previous_time = 0;
    for index = 1:nVacuumLocation
        index_in_results = vacuum_location(index);
        time = results.timestamps(index_in_results);
        interval = time - previous_time;
        previous_time = time;
        simulated_vacuumed_location_time(index, loc_index) = seconds(interval);
    end
end

simulation_vacuum_times_all = vacuumed_all_time;

nChargesB0 = size(chargingB0_trans_indices, 1);
nChargesB1 = size(chargingB1_trans_indices, 1);

simulated_average_time_between_charges = running_time/(nChargesB0 + nChargesB1);

%% Real Execution Results

load("execution_results_second_run.mat", "results")

chargingB0_trans_indices = find(results.transitions == "Charge_BS_B0");
chargedB0_trans_indices = find(results.transitions == "Charged_BS_B0");
chargingB1_trans_indices = find(results.transitions == "Charge_BS_B1");
chargedB1_trans_indices = find(results.transitions == "Charged_BS_B1");

chargingB0_timestamps = results.timestamps(chargingB0_trans_indices);
chargedB0_timestamps = results.timestamps(chargedB0_trans_indices);
chargingB1_timestamps = results.timestamps(chargingB1_trans_indices);
chargedB1_timestamps = results.timestamps(chargedB1_trans_indices);

chargingB0_timestamps = chargingB0_timestamps(1:size(chargedB0_timestamps, 1));
chargingB1_timestamps = chargingB1_timestamps(1:size(chargedB1_timestamps, 1));

execution_chargingB0_durations = seconds(chargedB0_timestamps - chargingB0_timestamps);
execution_chargingB1_durations = seconds(chargedB1_timestamps - chargingB1_timestamps);

vacuumed_all_trans_indices = find(results.transitions == "VacuumedAll");
nVacuumedAll = size(vacuumed_all_trans_indices, 1);
vacuumed_all_time = zeros(nVacuumedAll, 1);

charging_timestamps = results.timestamps(chargingB0_trans_indices);

nTransitions_fired = size(results.transitions, 1);
running_time = seconds(results.timestamps(nTransitions_fired));
average_reward_real = results.reward/running_time;

previous_time = 0;

for index = 1:nVacuumedAll
    index_in_results = vacuumed_all_trans_indices(index);
    time = results.timestamps(index_in_results);
    interval = time - previous_time;
    previous_time = time;
    vacuumed_all_time(index) = seconds(interval);
end

real_vacuum_times_all = vacuumed_all_time;

vacuuming_locations = ["L2", "L3", "L4", "L5", "L6"];
nVacuumingLocations = size(vacuuming_locations, 2);

for loc_index = 1:nVacuumingLocations
    location = vacuuming_locations(loc_index);
    start_string = "Vacuum_"+location;
    vacuum_location = find(startsWith(results.transitions, start_string));
    nVacuumLocation = size(vacuum_location, 1);
    previous_time = 0;
    for index = 1:nVacuumLocation
        index_in_results = vacuum_location(index);
        time = results.timestamps(index_in_results);
        interval = time - previous_time;
        previous_time = time;
        real_vacuumed_location_time(index, loc_index) = seconds(interval);
    end
end

execution_vacuum_times_all = vacuumed_all_time;

nChargesB0 = size(chargingB0_trans_indices, 1);
nChargesB1 = size(chargingB1_trans_indices, 1);

real_average_time_between_charges = running_time/(nChargesB0 + nChargesB1);


