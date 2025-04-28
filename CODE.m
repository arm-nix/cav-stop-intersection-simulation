clc; clear;
pause_sim = false;

% Constants
lane_width = 12;  % in feet
offset = 4;       % in feet
L = 100;          % segment length in ft (default)
veh_length = 14;
veh_width = 6;

% User Inputs
num_approaches = 2;
num_lanes = zeros(1, num_approaches);
vehicle_counts = cell(1, num_approaches);
fprintf('--- User Input ---\n');
% --- Initialize and Get Vehicle Counts ---
for p = 1:num_approaches
    lane_input = input(sprintf('Enter number of lanes for approach %d (default 2): ', p), 's');
    if isempty(lane_input)
        num_lanes(p) = 2;
    else
        num_lanes(p) = str2double(lane_input);
    end
    vehicle_counts{p} = zeros(1, num_lanes(p));
    for ln = 1:num_lanes(p)
        veh_input = input(sprintf('Enter number of vehicles in lane %d of approach %d (default 5): ', ln, p), 's');
        if isempty(veh_input)
            vehicle_counts{p}(ln) = 5;
        else
            vehicle_counts{p}(ln) = str2double(veh_input);
        end
    end
end

vehicle_A = sum(vehicle_counts{1});
vehicle_B = sum(vehicle_counts{2});
lanes_A = num_lanes(1);
lanes_B = num_lanes(2);

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
fprintf('\n========== HUMAN-DRIVEN SCENARIO ==========\n');
final_block_human = simulate_traffic(arrival_times_human, vehicle_counts, num_lanes, v_human, a_min_human, delta_human, mod_tau_S_human, mod_tau_F_human);
efficiency_human = compute_efficiency(final_block_human, num_lanes);

% === RUN CAV-DRIVEN SIMULATION ===
fprintf('\n========== CAV-DRIVEN SCENARIO ==========\n');
final_block_cav = simulate_traffic(arrival_times_cav, vehicle_counts, num_lanes, v_cav, a_min_cav, delta_cav, mod_tau_S_cav, mod_tau_F_cav);
efficiency_cav = compute_efficiency(final_block_cav, num_lanes);

% === DISPLAY COMPARISON ===
fprintf('\n========== COMPARISON SUMMARY ==========\n');
fprintf('Human-Driven Efficiency: %.2f%%\n', efficiency_human);
fprintf('CAV-Driven Efficiency  : %.2f%%\n', efficiency_cav);

% === EXPORT CSVs ===
writecell([{'Approach', 'Lane', 'Vehicle#', 'ArrivalTime', 'StopTime', 'DepartureTime'}; num2cell(final_block_human)], 'human_final_sequence.csv');
writecell([{'Approach', 'Lane', 'Vehicle#', 'ArrivalTime', 'StopTime', 'DepartureTime'}; num2cell(final_block_cav)], 'cav_final_sequence.csv');
fprintf('\nData saved to final_human.csv and final_cav.csv\n');

% === ANIMATED SEQUENCES ===
% === Side-by-Side Animated Timeline Comparison ===
figure('Name', 'Animated Timeline Comparison', 'NumberTitle', 'off', ...
             'Units', 'normalized', 'OuterPosition', [0 0 1 1]);

plotAnimatedTimeline(final_block_human, 'Human-Driven', 1,tau_F_human, tau_S_human, mod_tau_F_human, mod_tau_S_human, v_human);
plotAnimatedTimeline(final_block_cav, 'CAV-Driven', 2,tau_F_cav, tau_S_cav, mod_tau_F_cav, mod_tau_S_cav, v_cav);

pause(5);
% === Bird's Eye View Comparison ===
figure('Name', 'Bird''s Eye View Comparison', 'NumberTitle', 'off', ...
       'Units', 'normalized', 'OuterPosition', [0 0 1 1]);

simulateBirdsEyeView({final_block_human, final_block_cav}, lanes_A, lanes_B, [v_human, v_cav], a_max_human, {'Human-Driven', 'CAV-Driven'},box_length,box_width);

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

% --- Collect and Display Results ---
fprintf('\n--- Final Passing Sequence (All Approaches) ---\n');

results = []; % [Approach, Lane, Vehicle#, Arrival, Stop, Departure]

for p = 1:num_approaches
    for i = 1:total_vehicles(p)
        results(end+1, :) = [p, vehicle_lane_map{p}(i), vehicle_ids{p}(i), ...
            arrival_times{p}(i), stop_times{p}(i), departure_times{p}(i)];
    end
end

% Sort by departure time
results = sortrows(results, 6)
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

% Display the blocks
fprintf('\n--- Approach‑Headway Blocks (tau_S = %.2f s) ---\n', tau_S);
for k = 1:blkCount
    fprintf('\nBlock %d (size %d):\n', k, size(blocks{k},1));
    disp(blocks{k});
end

% Start with block 1
final_block = blocks{1};

% --- Merge remaining blocks into final_block ---
i = 2;
while i <= blkCount
    curr_block = blocks{i};
    last_main = final_block(end, :);  % Last vehicle in the final block
    
    % Default: no delay
    block_delay = 0;
    
    % --- Case: Different approach → check switching headway (tau_S) for each vehicle in curr_block ---
    for j = 1:size(curr_block, 1)
        first_curr = curr_block(j, :);  % Current vehicle
        
        % Check if vehicle is from a different approach (i.e., switching)
        if first_curr(1) ~= last_main(1)  % Opposite approach
            required_start = last_main(6) + tau_S;  % Minimum departure time for the current vehicle
            actual_start = first_curr(6);  % Actual departure time of the current vehicle
            
            % If the departure time is too early, calculate the delay needed
            if actual_start < required_start
                block_delay = required_start - actual_start;
                % Adjust the departure time of the vehicle
                curr_block(j, 6) = curr_block(j, 6) + block_delay;
            end
        end
    end

    % --- Enforce car-following constraint with prior vehicles in final_block ---
    for ln = unique(curr_block(:, 2))'  % Iterate through each lane in curr_block
        approach = curr_block(1, 1);  % Approach of the first vehicle in the block
        
        % Get the last vehicle in final_block with the same approach and lane
        last_idx = find(final_block(:, 1) == approach & final_block(:, 2) == ln, 1, 'last');
        
        if ~isempty(last_idx)
            last_dep = final_block(last_idx, 6);
            
            % Get indices of vehicles in curr_block that are in this lane
            lane_indices = find(curr_block(:, 2) == ln);
            for k = 1:length(lane_indices)
                idx = lane_indices(k);
                
                % Ensure car-following constraint (tau_F) is respected for stop time
                if curr_block(idx, 5) < last_dep + tau_F  % Stop time violates tau_F
                    curr_block(idx, 5) = last_dep + tau_F;
                    curr_block(idx, 6) = max(curr_block(idx, 6), curr_block(idx, 5));  % Ensure departure time is not earlier than stop time
                end
                last_dep = curr_block(idx, 6);  % Update rolling departure
            end
        end
    end
    
    % --- Enforce car-following *within* the curr_block itself ---
    for ln = unique(curr_block(:, 2))'  % Iterate through each lane in curr_block
        lane_indices = find(curr_block(:, 2) == ln);
        for k = 2:length(lane_indices)
            prev_idx = lane_indices(k-1);
            curr_idx = lane_indices(k);
            
            prev_dep = curr_block(prev_idx, 6);
            if curr_block(curr_idx, 5) < prev_dep + tau_F  % Check for tau_F violation within the same lane
                curr_block(curr_idx, 5) = prev_dep + tau_F;  % Adjust stop time
                curr_block(curr_idx, 6) = max(curr_block(curr_idx, 6), curr_block(curr_idx, 5));  % Ensure departure time
            end
        end
    end
    
    % Append processed block to final_block
    final_block = [final_block; curr_block];
    i = i + 1;
end

% Sort final_block by departure time to maintain proper sequence
final_block = sortrows(final_block, 6);

% Display final merged results
fprintf('\n--- Final Passing Order After Post-Processing ---\n');
disp(final_block);
end

function efficiency = compute_efficiency(final_block, num_lanes)
% --- Calculate Intersection Throughput Efficiency ---

% Total time range from first to last vehicle passing
start_time = final_block(1,6);  % first vehicle's departure
end_time   = final_block(end,6); % last vehicle's departure
total_time = end_time - start_time;

% Total number of vehicles processed
total_vehicles = size(final_block, 1);

% Actual throughput: vehicles per second
actual_throughput = size(final_block, 1) / (final_block(end,6) - final_block(1,6));

% Standard AASHTO saturation flow rate assume stop-controlled or unsignalized rural intersection
saturation_flow_rate_hr = 1800;           % in veh/hr/lane
saturation_flow_rate = saturation_flow_rate_hr / 3600; % in veh/s/lane

% Estimate theoretical capacity (assume dominant approach governs)
max_lanes = max(num_lanes);  % max number of lanes among approaches
max_throughput = saturation_flow_rate * max_lanes;  % veh/s

% Efficiency
efficiency = actual_throughput / max_throughput * 100;


% --- Display ---
fprintf('\n--- Throughput Efficiency Metrics ---\n');
fprintf('Total vehicles processed      : %d\n', total_vehicles);
fprintf('Total time (s)                : %.2f\n', total_time);
fprintf('Actual throughput (veh/s)     : %.3f\n', actual_throughput);
fprintf('Max possible throughput (veh/s): %.3f\n', max_throughput);
fprintf('Efficiency (%%)                : %.2f%%\n', efficiency);

% --- Export Final Sequence to CSV ---
filename = 'final_vehicle_sequence.csv';

% Add headers for clarity
headers = {'Approach', 'Lane', 'Vehicle#', 'ArrivalTime', 'StopTime', 'DepartureTime'};

% Combine headers and data
csv_data = [headers; num2cell(final_block)];

% Write to CSV file
writecell(csv_data, filename);

fprintf('\nFinal sequence exported to "%s"\n', filename);
end

% ---- Animated Timeline After Post-Processing ----
function plotAnimatedTimeline(final_block, scenario_title, subplot_idx, tau_F_original, tau_S_original, tau_F_mod, tau_S_mod, v)
subplot(1, 2, subplot_idx);
hold on;
title(['Timeline - ' scenario_title]);
xlabel('Time (seconds)');
ylabel('Vehicle ID');

% Generate vehicle labels like A1-1, B2-3, etc.
vehicle_labels = strings(size(final_block,1), 1);
for i = 1:size(final_block,1)
    approach_letter = char('A' + final_block(i,1) - 1);
    vehicle_labels(i) = sprintf('%s%d-%d', approach_letter, final_block(i,2), final_block(i,3));
end

% Sort final block by departure time for visualization
[~, sort_idx] = sort(final_block(:,6));
sorted_block = final_block(sort_idx, :);
vehicle_labels = vehicle_labels(sort_idx);
results=sorted_block;

% Calculate average delay
average_delay = mean(sorted_block(:,6) - sorted_block(:,5));

% Setup axis limits and ticks
x_max = max(sorted_block(:,6)) + 25;
axis([-10, x_max, 0, size(sorted_block,1) + 1]);
yticks(1:size(sorted_block,1));
yticklabels(vehicle_labels);
grid on;

% Initialize animated plot
arrival_handles = gobjects(size(sorted_block,1), 1);
stop_handles = gobjects(size(sorted_block,1), 1);
depart_handles = gobjects(size(sorted_block,1), 1);

for i = 1:size(sorted_block,1)
    arrival = sorted_block(i,4);
    stop = sorted_block(i,5);
    depart = sorted_block(i,6);
    approach = sorted_block(i,1);

    % Assign approach-specific color
    if approach == 1
        color = [0.2, 0.4, 1.0]; % blue for A
    else
        color = [1.0, 0.3, 0.3]; % red for B
    end

    % Plot Arrival Marker
    arrival_handles(i) = plot(arrival, i, 'o', 'Color', color, ...
        'MarkerSize', 6, 'MarkerFaceColor', color);
    pause(0.05);

        % Plot Stop Marker
    stop_handles(i) = plot(stop, i, 'd', 'Color', color, ...
        'MarkerSize', 6, 'MarkerFaceColor', color);
    pause(0.05);

    % Plot Departure Marker
    depart_handles(i) = plot(depart, i, '>', 'Color', color * 0.8, ...
        'MarkerSize', 6, 'MarkerFaceColor', color);
    pause(0.05);

    % Plot Delta Line (Stop to Departure)
    delta_handles(i) = line([arrival, stop], [i, i], ...
        'Color', color, 'LineStyle', '-', 'LineWidth', 2);
    pause(0.05);
    
    % Plot Delay Line (Stop to Departure)
    delay_handles(i) = line([stop, depart], [i, i], ...
        'Color', color*0.8, 'LineStyle', '-', 'LineWidth', 2);
    pause(0.05);
    
    % Add Annotations
    text(arrival - 0.5, i, sprintf('Arr: %.1f    ', arrival), ...
        'HorizontalAlignment', 'right', 'FontSize', 7, 'Color', color);
    text(stop, i - 0.25, sprintf('Stp: %.1f', stop), ...
            'HorizontalAlignment', 'center', 'FontSize', 7, 'Color', [0.3 0.3 0.3]);
    text((stop + depart)/2, i + 0.3, sprintf('Delay: %.1f', depart - stop), ...
        'HorizontalAlignment', 'center', 'FontSize', 7, 'Color', [0.2 0.2 0.2]);
    text(depart + 0.5, i, sprintf('    Dep: %.1f', depart), ...
        'HorizontalAlignment', 'left', 'FontSize', 7, 'Color', color*0.8);
end

legend({'Arrival', 'Stop', 'Departure'}, 'Location', 'southeast');
% Display additional simulation parameters
stats_text = sprintf(['Total Vehicles: %d\n', ...
                      'Original \\tau_F = %.2f s\n','Modified \\tau_F = %.2f s\n', ...
                      'Original \\tau_S = %.2f s\n','Modified \\tau_S = %.2f s\n', ...
                      'v = %.1f ft/s\nAvg Delay = %.2f s'], ...
                      size(sorted_block,1), tau_F_original, tau_F_mod, ...
                      tau_S_original, tau_S_mod, v, average_delay);
x_stats = x_max-5; % Position stats text near the right edge
y_stats = size(sorted_block,1); 
text(x_stats, y_stats, stats_text, 'FontSize', 8, ...
     'VerticalAlignment', 'middle', 'BackgroundColor', [1 1 1 0.8], ...
     'EdgeColor', [0.6 0.6 0.6], 'Margin', 2);
end

% ==== Video Simulation: Side-by-Side Bird's Eye View Comparison ====
function simulateBirdsEyeView(results_cell, lanes_A, lanes_B, v_vals, a_max, scenario_titles,box_length,box_width)

lane_width = 12;
veh_length = 14;
veh_width = 6;
min_gap = veh_length;
stop_line_A = -100 - (lane_width*lanes_B)/2 - 4 - veh_length/2;
stop_line_B = 100 + (lane_width*lanes_A)/2 + 4 + veh_length/2;
intersection_width = lane_width * max(lanes_A, lanes_B) + 8;
fps = 3;
dt = 1 / fps;

vehicle_objs = cell(1, 2);
vehicle_queues = cell(1, 2);

% Create pause button once
pause_btn = uicontrol('Style', 'togglebutton', ...
                      'String', 'Pause', ...
                      'Position', [20, 20, 80, 30], ...
                      'Callback', @(src,~) togglePause(src));
function togglePause(src)
    if src.Value
        src.String = 'Resume';
    else
        src.String = 'Pause';
    end
end

% Setup and initialization for both views
for view_idx = 1:2
    subplot(1, 2, view_idx);
    title(['Bird''s Eye View - ' scenario_titles{view_idx}]);
    hold on;
    axis equal;
    axis([-250 50 -50 250]);
    xlabel('X (feet)');
    ylabel('Y (feet)');
    set(gca, 'Color', [0.9 0.9 0.9]);
    grid on;
    % Draw colored boxes as legend for approaches
    rectangle('Position', [-240, 210, 10, 10], 'FaceColor', [0.2, 0.4, 1.0], 'EdgeColor', 'k');
    text(-225, 215, 'Approach A', 'FontSize', 9, 'FontWeight', 'bold', 'Color', 'k');
    
    rectangle('Position', [-240, 190, 10, 10], 'FaceColor', [1.0, 0.3, 0.3], 'EdgeColor', 'k');
    text(-225, 195, 'Approach B', 'FontSize', 9, 'FontWeight', 'bold', 'Color', 'k');

    % Lane guides
    for i = 1:lanes_A
        y = 100 + (i - (lanes_A + 1)/2) * lane_width;
        plot([-235 50], [y y], 'b--');
    end
    for i = 1:lanes_B
        x = -100 + (i - (lanes_B + 1)/2) * lane_width;
        plot([x x], [-50 245], 'r--');
    end
        % Add lane labels
    for i = 1:lanes_A
        y = 100 + (i - (lanes_A + 1)/2) * lane_width;
        text(-235, y, sprintf('A-%d', i), ...
             'FontSize', 8, 'FontWeight', 'bold', ...
             'HorizontalAlignment', 'right', 'Color', [0.2, 0.4, 1.0]);
    end
    for i = 1:lanes_B
        x = -100 + (i - (lanes_B + 1)/2) * lane_width;
        text(x, 245, sprintf('B-%d', i), ...
             'FontSize', 8, 'FontWeight', 'bold', ...
             'HorizontalAlignment', 'center', 'Color', [1.0, 0.3, 0.3]);
    end

    rectangle('Position', [-100 - (lane_width*lanes_B)/2 - 4, ...
                           100 - (lane_width*lanes_A)/2 - 4, ...
                           lane_width*lanes_B + 8, ...
                           lane_width*lanes_A + 8], ...
              'EdgeColor', 'k', 'LineStyle', '--', 'LineWidth', 1.5);

    % Timestamp display
    time_texts(view_idx) = text(90, 0, 'Time: 0.0 s', ...
                                'FontSize', 10, 'FontWeight', 'bold', ...
                                'HorizontalAlignment', 'right', ...
                                'BackgroundColor', 'w');

    results = results_cell{view_idx};
    v = v_vals(view_idx);
    objs = containers.Map();
    queues = struct();

    for i = 1:size(results,1)
        app = results(i,1);
        lane = results(i,2);
        vnum = results(i,3);
        arr = results(i,4);
        stp = results(i,5);
        dpt = results(i,6);
        v_id = sprintf('%d_%d_%d', app, lane, vnum);
        obj_key = matlab.lang.makeValidName(v_id);

        if app == 1
            lane_offset = lane_width * (lane - (lanes_A + 1)/2);
            stop_pos = [stop_line_A, 100 + lane_offset];
            arrival_pos = [stop_line_A - 100, 100 + lane_offset];
            dir = [1, 0];
            color = [0.2, 0.4, 1.0];
            shape = [veh_length, veh_width];
        else
            lane_offset = lane_width * (lane - (lanes_B + 1)/2);
            stop_pos = [-100 + lane_offset, stop_line_B];
            arrival_pos = [-100 + lane_offset, stop_line_B + 100];
            dir = [0, -1];
            color = [1.0, 0.3, 0.3];
            shape = [veh_width, veh_length];
        end

        patch_h = rectangle('Position', ...
            [arrival_pos(1)-shape(1)/2, arrival_pos(2)-shape(2)/2, shape(1), shape(2)], ...
            'FaceColor', color, 'EdgeColor', 'k', 'LineWidth', 1.5, 'Visible', 'off');

        rotation_angle = (app == 2) * 90;
        label_h = text(arrival_pos(1), arrival_pos(2), ...
            sprintf('%s%d-%d', char('A'+app-1), lane, vnum), ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
            'FontSize', 5, 'FontWeight', 'bold', ...
            'Color', [0.1 0.1 0.1], 'BackgroundColor', 'w', 'Margin', 1, ...
            'Rotation', rotation_angle, 'Visible', 'off');

        objs(obj_key) = struct('patch', patch_h, 'label', label_h, ...
            'arrival', arr, 'stop', stp, 'depart', dpt, ...
            'arrival_pos', arrival_pos, 'stop_pos', stop_pos, 'dir', dir, ...
            'pos', arrival_pos, 'approach', app, 'shape', shape);

        qkey = sprintf('a%d_l%d', app, lane);
        if isfield(queues, qkey)
            queues.(qkey){end+1} = v_id;
        else
            queues.(qkey) = {v_id};
        end
    end

    vehicle_objs{view_idx} = objs;
    vehicle_queues{view_idx} = queues;
end

% Determine sim duration
% Determine individual max departure times
if(results_cell{1}(size(results_cell{1},1),1))==1
    t=box_length/v_vals(view_idx);
else t=box_width/v_vals(view_idx);
end

max_depart_time = [ ...
    max(results_cell{1}(:,6)), ...
    max(results_cell{2}(:,6)) ...
];
% === Initialize video writer ===
video_filename = 'BirdsEyeViewComparison.mp4';  % or .avi
v_writer = VideoWriter(video_filename, 'MPEG-4');  % Use 'Motion JPEG AVI' for .avi
v_writer.FrameRate = fps;  % match your simulation FPS
open(v_writer);

% Animation loop
for t = 0:dt:max_depart_time+5
    for view_idx = 1:2
        if t <= max_depart_time(view_idx)
            set(time_texts(view_idx), 'String', sprintf('Time: %.1f s', t));
        end
    end

    while get(pause_btn, 'Value')
        pause(0.1);
    end

    for view_idx = 1:2
        subplot(1, 2, view_idx);
        objs = vehicle_objs{view_idx};
        queues = vehicle_queues{view_idx};
        v = v_vals(view_idx);

        keys = fieldnames(queues);
        for i = 1:length(keys)
            queue = queues.(keys{i});
            for k = 1:length(queue)
                vid = queue{k};
                obj_key = matlab.lang.makeValidName(vid);
                if ~isKey(objs, obj_key), continue; end

                obj = objs(obj_key);
                if k == 1
                    target = obj.stop_pos;
                else
                    prev_key = matlab.lang.makeValidName(queue{k-1});
                    if isKey(objs, prev_key)
                        prev_obj = objs(prev_key);
                        if t < obj.stop && prev_obj.depart > t
                            if obj.approach == 1
                                target = [prev_obj.pos(1) - obj.shape(1) - min_gap, obj.stop_pos(2)];
                            else
                                target = [obj.stop_pos(1), prev_obj.pos(2) + obj.shape(2) + min_gap];
                            end
                        else
                            target = obj.stop_pos;
                        end
                    else
                        target = obj.stop_pos;
                    end
                end

                if obj.approach == 1
                    target(1) = min(target(1), stop_line_A);
                else
                    target(2) = max(target(2), stop_line_B);
                end

                if t < obj.arrival
                    new_pos = obj.arrival_pos;
                elseif t < obj.stop
                    travel_time = obj.stop - obj.arrival;
                    travel_dist = norm(obj.stop_pos - obj.arrival_pos);
                    travel_speed = travel_dist / travel_time;
                    d = min((t - obj.arrival) * travel_speed, travel_dist);
                    direction_unit = (obj.stop_pos - obj.arrival_pos) / norm(obj.stop_pos - obj.arrival_pos);
                    new_pos = obj.arrival_pos + d * direction_unit;
                elseif t < obj.depart
                    new_pos = target;
                else
                    d = v * (t - obj.depart);
                    new_pos = target + d * obj.dir;
                end

                obj.pos = new_pos;
                if t >= obj.arrival
                    set(obj.patch, 'Position', ...
                        [new_pos(1) - obj.shape(1)/2, new_pos(2) - obj.shape(2)/2, obj.shape(1), obj.shape(2)], ...
                        'Visible', 'on');
                    set(obj.label, 'Position', new_pos);
                    if new_pos(1) < -250 || new_pos(1) > 50 || new_pos(2) < -50 || new_pos(2) > 250
                        set(obj.label, 'Visible', 'off');
                    else
                        set(obj.label, 'Visible', 'on');
                    end
                else
                    set(obj.patch, 'Visible', 'off');
                    set(obj.label, 'Visible', 'off');
                end

                objs(obj_key) = obj;
            end
        end
        vehicle_objs{view_idx} = objs;
    end

    drawnow;
    pause(0.2);
    frame = getframe(gcf);  % capture current figure
    writeVideo(v_writer, frame);
end
close(v_writer);
disp(['Video saved as: ', video_filename]);
end
