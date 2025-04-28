clc; clear;

% === One-Time Configuration ===
pause_sim = false;
lane_width = 12;  % in feet
offset = 4;       % in feet
L = 100;          % segment length in ft
num_approaches = 2;
veh_length = 14;
veh_width = 6;

% Simulation Parameters
max_lanes = 20;
min_vehicles = 5;
max_vehicles = 100;
num_trials = input('\nEnter the number of data points to generate for the plot: ');

% --- Human Inputs ---
fprintf('\n--- Parameters for HUMAN‑DRIVEN Vehicles ---\n');
v_human = get_with_default('Enter human-driving posted speed in ft/s (default 45): ', 45);
a_min_human = get_with_default('Enter human-driving minimum acceleration (negative for deceleration) in ft/s^2 (default -5): ', -5);
a_max_human = get_with_default('Enter human-driving maximum acceleration in ft/s^2 (default 6.5): ', 6.5);
tau_S_human = get_with_default('Enter human-driving switching time headway (tau_S) in seconds (default 2.5): ', 2.5);
tau_F_human = get_with_default('Enter human-driving car-following headway (tau_F) in seconds (default 2): ', 2);
lambda_human = get_with_default('Enter human-driving lambda value (default 0.3): ', 0.3);

% --- CAV Inputs ---
fprintf('\n--- Parameters for CAV‑DRIVEN Vehicles ---\n');
v_cav = get_with_default('Enter CAV posted speed in ft/s (default 60): ', 60);
a_min_cav = get_with_default('Enter CAV minimum acceleration (negative for deceleration) in ft/s^2 (default -15): ', -15);
a_max_cav = get_with_default('Enter CAV maximum acceleration in ft/s^2 (default 10): ', 10);
tau_S_cav = get_with_default('Enter CAV switching time headway (tau_S) in seconds (default 0.8): ', 0.8);
tau_F_cav = get_with_default('Enter CAV car-following headway (tau_F) in seconds (default 0.6): ', 0.6);
lambda_cav = get_with_default('Enter CAV lambda value (default 0.7): ', 0.7);

% --- Helper function ---
function val = get_with_default(prompt, default_val)
    str = input(prompt, 's');
    if isempty(str)
        val = default_val;
    else
        val = str2double(str);
    end
end

% Initialize Logs
conflict_points_log = zeros(num_trials, 1);
efficiency_gain_log = zeros(num_trials, 1);
lanesA_log = zeros(num_trials, 1);
lanesB_log = zeros(num_trials, 1);
vehiclesA_log = strings(num_trials, 1);
vehiclesB_log = strings(num_trials, 1);

% === MAIN LOOP ===
for iter = 1:num_trials
    % Random lane and vehicle setup
    num_lanes = randi([1, max_lanes], 1, num_approaches);
    vehicle_counts = cell(1, num_approaches);
    for p = 1:num_approaches
        vehicle_counts{p} = randi([min_vehicles, max_vehicles], 1, num_lanes(p));
    end

    % Derived Parameters
    delta_human = L/v_human + v_human/(-2 * a_min_human);  % segment travel time
    delta_cav = L/v_cav + v_cav/(-2 * a_min_cav);  % segment travel time
    box_length = lane_width * num_lanes(1) + 2 * offset;  % intersection box length
    box_width = lane_width * num_lanes(2) + 2 * offset;  % intersection box width
    %fprintf('\nIntersection box size: %.2f ft x %.2f ft\n', box_length, box_width);
    
    % --- Modified tau values ---
    mod_tau_S_human = max(tau_S_human, (veh_length + max(box_length, box_width)) / v_human);
    mod_tau_F_human = max(tau_F_human, 1.5 * veh_length / v_human);
    mod_tau_S_cav = max(tau_S_cav, (veh_length + max(box_length, box_width)) / v_cav);
    mod_tau_F_cav = max(tau_F_cav, 1.5*veh_length / v_cav);
    
    % Generate vehicle arrival times with a minimum time gap between consecutive arrivals
    arrival_times_human = generateArrivalTimes(vehicle_counts, lambda_human, 1.5*veh_length/v_human + v_human/(-2 * a_min_human));
    arrival_times_cav   = generateArrivalTimes(vehicle_counts, lambda_cav, 1.5*veh_length/v_cav + v_cav/(-2 * a_min_cav));
    
    % === RUN HUMAN-DRIVEN SIMULATION ===
    % fprintf('\n========== HUMAN-DRIVEN SCENARIO ==========\n');
    final_block_human = simulate_traffic(arrival_times_human, vehicle_counts, num_lanes, v_human, a_min_human, delta_human, mod_tau_S_human, mod_tau_F_human);
    efficiency_human = compute_efficiency(final_block_human, num_lanes);
    
    % === RUN CAV-DRIVEN SIMULATION ===
    % fprintf('\n========== CAV-DRIVEN SCENARIO ==========\n');
    final_block_cav = simulate_traffic(arrival_times_cav, vehicle_counts, num_lanes, v_cav, a_min_cav, delta_cav, mod_tau_S_cav, mod_tau_F_cav);
    efficiency_cav = compute_efficiency(final_block_cav, num_lanes);



    % Metrics
    efficiency_gain = ((efficiency_cav - efficiency_human) / efficiency_human) * 100;
    conflict_points = num_lanes(1) * num_lanes(2);

    % Store Results
    conflict_points_log(iter) = conflict_points;
    efficiency_gain_log(iter) = efficiency_gain;
    lanesA_log(iter) = num_lanes(1);
    lanesB_log(iter) = num_lanes(2);
    vehiclesA_log(iter) = strjoin(string(vehicle_counts{1}), '-');
    vehiclesB_log(iter) = strjoin(string(vehicle_counts{2}), '-');

    % Status
    fprintf('Trial %d/%d: Conflict Points = %d | Efficiency Gain = %.2f%%\n', ...
            iter, num_trials, conflict_points, efficiency_gain);
end

% === Save Results ===
results_table = table(lanesA_log, lanesB_log, vehiclesA_log, vehiclesB_log, ...
                      conflict_points_log, efficiency_gain_log, ...
                      'VariableNames', {'LanesA', 'LanesB', 'VehiclesA', 'VehiclesB', ...
                                        'ConflictPoints', 'EfficiencyGain'});
writetable(results_table, 'efficiency_vs_conflict_points.csv');
fprintf('\nResults saved to "efficiency_vs_conflict_points.csv"\n');

% === Plot ===
figure;
scatter(conflict_points_log, efficiency_gain_log, 100, 'filled');
grid on;
xlabel('Conflict Points (lanes A × lanes B)');
ylabel('% Efficiency Increase (CAV vs Human)');
title('Efficiency Gain vs Conflict Points');

function [num_lanes, vehicle_counts] = getLaneConfig(num_approaches)
    num_lanes = zeros(1, num_approaches);
    vehicle_counts = cell(1, num_approaches);
    for p = 1:num_approaches
        num_lanes(p) = input(sprintf('Enter number of lanes for approach %d: ', p));
        vehicle_counts{p} = zeros(1, num_lanes(p));
        for ln = 1:num_lanes(p)
            vehicle_counts{p}(ln) = input(sprintf('Enter number of vehicles in lane %d of approach %d: ', ln, p));
        end
    end
end

function arrival_times = generateArrivalTimes(vehicle_counts, lambda, min_gap_time)
    num_approaches = length(vehicle_counts);
    arrival_times = cell(1, num_approaches);

    for p = 1:num_approaches
        lane_counts = vehicle_counts{p};         % e.g. [3 2 4]
        total_vehicles = sum(lane_counts);       % total number of vehicles
        num_lanes = length(lane_counts);         % number of lanes

        % === Step 1: Generate random lane assignments ===
        lane_list = [];
        for lane = 1:num_lanes
            lane_list = [lane_list; repmat(lane, lane_counts(lane), 1)];
        end
        lane_list = lane_list(randperm(total_vehicles));  % Shuffle lanes

        % === Step 2: Generate inter-arrival times ===
        inter_arrivals = max(-log(rand(total_vehicles, 1)) / lambda, min_gap_time);

        % === Step 3: Get absolute arrival times ===
        abs_arrivals = cumsum(inter_arrivals);

        % === Step 4: Sort final arrival events by time ===
        all_data = [abs_arrivals, lane_list];
        all_data = sortrows(all_data, 1);  % sort by arrival time

        arrival_times{p} = all_data;
    end
end

function final_block = simulate_traffic(arrival_times, vehicle_counts, num_lanes, v, a_min, delta, tau_S, tau_F)
num_approaches = length(arrival_times);
total_vehicles = cellfun(@sum, vehicle_counts);
stop_times = cell(1, num_approaches);
departure_times = cell(1, num_approaches);
next_vehicle = ones(1, num_approaches);
last_departure = -inf(1, num_approaches);
p_prev = 1;

% Maintain per-lane queues
max_lanes = max(cellfun(@length, vehicle_counts));
lane_queues = cell(num_approaches, max(cellfun(@max, vehicle_counts)));
last_depart_lane = -inf(num_approaches, max(cellfun(@max, vehicle_counts)));

% Assign lane info
vehicle_lane_map = cell(1, num_approaches);
vehicle_ids = cell(1, num_approaches);
for p = 1:num_approaches
    total_p = total_vehicles(p);
    lane_assignments = arrival_times{p}(:,2);
    arrival_times{p} = arrival_times{p}(:,1);  % keep only arrival times

    vehicle_lane_map{p} = lane_assignments;
    stop_times{p} = zeros(total_p,1);
    departure_times{p} = zeros(total_p,1);

    % Reassign vehicle IDs based on order *within lane*
    vehicle_ids{p} = zeros(total_p, 1);
    for ln = 1:num_lanes(p)
        lane_indices = find(lane_assignments == ln);
        [~, sort_idx] = sort(arrival_times{p}(lane_indices));  % sort by arrival
        vehicle_ids{p}(lane_indices(sort_idx)) = 1:length(lane_indices);  % assign lane-local IDs
    end
end

% Process all vehicles
vehicles_remaining = sum(total_vehicles);
while vehicles_remaining > 0
    min_time = inf;
    p_next = -1;

    % Determine the next vehicle that can stop earliest
    for p = 1:num_approaches
        i = next_vehicle(p);
        if i <= total_vehicles(p)
            ln = vehicle_lane_map{p}(i);
            t_arrival = arrival_times{p}(i);

            % Car-following headway only within same lane
            t_stop = max(t_arrival + delta, last_depart_lane(p, ln) + tau_F);

            stop_times{p}(i) = t_stop;
            if t_stop < min_time
                min_time = t_stop;
                p_next = p;
            end
        end
    end

    % Schedule departure for selected vehicle
    i = next_vehicle(p_next);
    ln = vehicle_lane_map{p_next}(i);
    t_stop = stop_times{p_next}(i);

    if p_next == p_prev
        t_depart = t_stop;
    else
        t_depart = max(t_stop, last_departure(p_prev) + tau_S);
    end

    departure_times{p_next}(i) = t_depart;
    last_departure(p_next) = t_depart;
    last_depart_lane(p_next, ln) = t_depart;

    next_vehicle(p_next) = next_vehicle(p_next) + 1;
    p_prev = p_next;
    vehicles_remaining = vehicles_remaining - 1;
end

% % --- Collect and Display Results ---
% fprintf('\n--- Final Passing Sequence (All Approaches) ---\n');

results = []; % [Approach, Lane, Vehicle#, Arrival, Stop, Departure]

for p = 1:num_approaches
    for i = 1:total_vehicles(p)
        results(end+1, :) = [p, vehicle_lane_map{p}(i), vehicle_ids{p}(i), ...
            arrival_times{p}(i), stop_times{p}(i), departure_times{p}(i)];
    end
end

% Sort by departure time
results = sortrows(results, 6);
temp = results;
% --- Post‑process for switching‑headway blocks ---
% temp is N×6 matrix: [Approach, Lane, Vehicle#, Arrival, Stop, Departure]
numVeh = size(temp,1);

% Pre‑allocate cell array for blocks
blocks   = cell(1,1);
blkCount = 1;
blocks{blkCount} = temp(1,:);   % start block 1 with the very first vehicle

for i = 2:numVeh
    curr = temp(i,:);
    prev = temp(i-1,:);
    
    % same approach AND headway gap less than tau_S?
    if curr(1) == prev(1) && (curr(6) - prev(6)) < tau_S
        % → stay in current block
        blocks{blkCount}(end+1, :) = curr;
    else
        % → start a new block
        blkCount = blkCount + 1;
        blocks{blkCount} = curr;
    end
end

% % Display the blocks
% fprintf('\n--- Approach‑Headway Blocks (tau_S = %.2f s) ---\n', tau_S);
% for k = 1:blkCount
%     fprintf('\nBlock %d (size %d):\n', k, size(blocks{k},1));
%     disp(blocks{k});
% end

% Start with block 1
final_block = blocks{1};

% --- Merge remaining blocks into final_block ---
i = 2;
while i <= blkCount
    curr_block = blocks{i};
    last_main = final_block(end, :);  % Last vehicle in the final block
    
    % Default: no delay
    block_delay = 0;
    
    % --- Check for switching headway (tau_S) for each vehicle in curr_block ---
    for j = 1:size(curr_block, 1)
        first_curr = curr_block(j, :);  % Current vehicle
        
        % Only if approaches are different (switching)
        if first_curr(1) ~= last_main(1)
            required_start = last_main(6) + tau_S;  % Minimum required depart time
            actual_start = first_curr(6);
            
            if actual_start < required_start
                block_delay = required_start - actual_start;
                curr_block(j, 6) = curr_block(j, 6) + block_delay;  % Adjust depart time
            end
        end
        last_main = first_curr;  % Update for next vehicle comparison
    end
    
    % --- Enforce car-following constraint (tau_F) with final_block ---
    for ln = unique(curr_block(:, 2))'  % Iterate through each lane
        approach = curr_block(1, 1);  % Approach of curr_block
        
        % Find last vehicle in final_block with same approach and lane
        last_idx = find(final_block(:, 1) == approach & final_block(:, 2) == ln, 1, 'last');
        
        if ~isempty(last_idx)
            last_dep = final_block(last_idx, 6);
            
            % Get vehicles in curr_block for this lane
            lane_indices = find(curr_block(:, 2) == ln);
            for k = 1:length(lane_indices)
                idx = lane_indices(k);
                
                % Car-following check (tau_F) on stop times
                if curr_block(idx, 5) < last_dep + tau_F
                    curr_block(idx, 5) = last_dep + tau_F;
                    curr_block(idx, 6) = max(curr_block(idx, 6), curr_block(idx, 5));
                end
                last_dep = curr_block(idx, 6);  % Update rolling departure
            end
        end
    end
    
    % --- Enforce car-following (tau_F) within curr_block itself ---
    for ln = unique(curr_block(:, 2))'
        lane_indices = find(curr_block(:, 2) == ln);
        for k = 2:length(lane_indices)
            prev_idx = lane_indices(k-1);
            curr_idx = lane_indices(k);
            
            prev_dep = curr_block(prev_idx, 6);
            if curr_block(curr_idx, 5) < prev_dep + tau_F
                curr_block(curr_idx, 5) = prev_dep + tau_F;
                curr_block(curr_idx, 6) = max(curr_block(curr_idx, 6), curr_block(curr_idx, 5));
            end
        end
    end
    
    % --- Check if reshuffling needed (only if approaches are different) ---
    if i < blkCount
        next_block = blocks{i+1};
        
        if ~isempty(next_block)
            first_curr = curr_block(1, :);    % First vehicle of current block
            first_next = next_block(1, :);     % First vehicle of next block
            
            if first_curr(1) ~= first_next(1)  % Only consider reshuffle if approaches are different
                if first_curr(6) > first_next(6)
                    % --- Perform reshuffle ---
                    temp = curr_block;
                    curr_block = next_block;
                    blocks{i+1} = temp;
                    
                    % After reshuffle, need to recheck switching headway for reshuffled curr_block
                    last_main = final_block(end, :);  % Update last_main
                    for j = 1:size(curr_block, 1)
                        first_curr = curr_block(j, :);
                        if first_curr(1) ~= last_main(1)
                            required_start = last_main(6) + tau_S;
                            actual_start = first_curr(6);
                            if actual_start < required_start
                                block_delay = required_start - actual_start;
                                curr_block(j, 6) = curr_block(j, 6) + block_delay;
                            end
                        end
                        last_main = first_curr;
                    end
                    
                    % Also enforce tau_F again within reshuffled curr_block
                    for ln = unique(curr_block(:, 2))'
                        lane_indices = find(curr_block(:, 2) == ln);
                        for k = 2:length(lane_indices)
                            prev_idx = lane_indices(k-1);
                            curr_idx = lane_indices(k);
                            
                            prev_dep = curr_block(prev_idx, 6);
                            if curr_block(curr_idx, 5) < prev_dep + tau_F
                                curr_block(curr_idx, 5) = prev_dep + tau_F;
                                curr_block(curr_idx, 6) = max(curr_block(curr_idx, 6), curr_block(curr_idx, 5));
                            end
                        end
                    end
                end
            end
        end
    end
    
    % --- Append processed curr_block to final_block ---
    final_block = [final_block; curr_block];
    i = i + 1;
end

% Sort final_block by departure time to maintain proper sequence
final_block = sortrows(final_block, 6);

% Display final merged results
% fprintf('\n--- Final Passing Order After Post-Processing ---\n');
% disp(final_block);
end

function efficiency = compute_efficiency(blocks, num_lanes)
% --- Calculate Intersection Throughput Efficiency ---

% Extract departure times (column 6)
all_departures = blocks(:, 6);

% Sort all departure times
all_departures = sort(all_departures);

% Ensure there are enough vehicles to compute meaningful efficiency
if length(all_departures) < 2
    efficiency = 0;
    return;
end

% Total time range from first to last vehicle passing
start_time = all_departures(1);
end_time   = all_departures(end);
total_time = end_time - start_time;

% Total number of vehicles processed
total_vehicles = length(all_departures);

% Actual throughput: vehicles per second
actual_throughput = total_vehicles / total_time;

% Standard AASHTO saturation flow rate assume stop-controlled or unsignalized rural intersection
saturation_flow_rate_hr = 1800;                       % in veh/hr/lane
saturation_flow_rate = saturation_flow_rate_hr / 3600; % in veh/s/lane

% Estimate theoretical capacity (assume dominant approach governs)
max_lanes = max(num_lanes);  % max number of lanes among approaches
max_throughput = saturation_flow_rate * max_lanes;  % veh/s

% Efficiency (%)
efficiency = (actual_throughput / max_throughput) * 100;
end
