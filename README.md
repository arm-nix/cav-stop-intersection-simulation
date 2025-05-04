# cav-stop-intersection-simulation
MATLAB simulation for vehicle arrival, scheduling, and headway constraint enforcement at intersections. Models randomized arrivals, lane assignment, and groups vehicles into switching and car-following blocks for realistic traffic flow.
This project simulates vehicle arrivals and scheduling at a signalized intersection, enforcing headway constraints to model realistic traffic flow.
Run CODE.m for a single illustrative example comparing one HDV vs CAV scenario. Run trial_submission.m to generate scatter plot data for randomised multiple comparison scenarios for a given set of HDV and CAV parameters.
Dependencies: MATLAB R2021a or later
Open MATLAB and set the working directory to the repository folder and run the codes.
The main outputs are: 
1) For CODE.m:
* **Console Output:**
    * Displays the calculated **Throughput Efficiency Metrics** for both the Human-Driven Vehicle (HDV) and Connected Automated Vehicle (CAV) scenarios. This includes:
        * `Total vehicles processed`: The number of vehicles simulated.
        * `Total time (s)`: The simulation duration from the first vehicle departure to the last.
        * `Actual throughput (veh/s)`: Calculated throughput based on total vehicles and total time.
        * `Max possible throughput (veh/s)`: A theoretical benchmark based on standard saturation flow rates and the number of lanes.
        * `Efficiency (%)`: The primary metric, calculated as `(Actual throughput / Max possible throughput) * 100`.
    * Prints a **Comparison Summary** showing the final efficiency percentages for HDV and CAV runs side-by-side.

* **CSV Files:**
    * `human_final_sequence.csv`: Contains the detailed final departure schedule for the HDV scenario after all post-processing adjustments.
    * `cav_final_sequence.csv`: Contains the detailed final departure schedule for the CAV scenario.
    * **Columns in CSV:** Approach (1 or 2), Lane Number, Vehicle Number (within that lane), Arrival Time ($t^-$ at start of segment), Stop Time ($t^0$ at stop line), Departure Time ($t^+$ from stop line). All times are in seconds. This data allows for in-depth analysis of vehicle timings and delays.

* **Plot Figure 1: Timeline Comparison:**
    * Shows two subplots comparing the HDV and CAV timelines.
    * Each vehicle is represented by a horizontal line showing its progression over time (Arrival 'o', Stop 'd', Departure '>').
    * The length of the line between the Stop diamond and Departure triangle visually represents the stopped delay.
    * Useful for comparing overall simulation duration and individual vehicle waiting times between scenarios.
    * A summary box provides key parameters ($\tau_F', \tau_S', v$) and the average stopped delay.

* **Plot Figure 2 / Video File (`BirdsEyeViewComparison.mp4`):**
    * Shows a top-down animated view of the intersection for both scenarios simultaneously.
    * Vehicles (colored rectangles) move according to the calculated schedule.
    * Helps visualize traffic flow and the spatial interactions between vehicles.

2) For trial_submission.m (based on CODE.m's core logic but with modified outputs, runs multiple randomized trials):
    * * **Console Output:**
    * Prints the status of each trial as it runs, showing the trial number, the randomly generated conflict points (`lanes A * lanes B`), and the calculated percentage efficiency gain for that trial.

* **CSV File (`efficiency_vs_conflict_points.csv`):**
    * This file stores the results from all trials.
    * **Columns:** `LanesA`, `LanesB` (number of lanes for each approach in the trial), `VehiclesA`, `VehiclesB` (string showing vehicles per lane, e.g., "5-5"), `ConflictPoints`, `EfficiencyGain` (percentage gain of CAV over HDV efficiency for that trial).
    * This is the primary data source for analyzing the relationship between complexity and efficiency gain.

* **Plot Figure 3: Scatter Plot:**
    * Visualizes the data from `efficiency_vs_conflict_points.csv`.
    * Plots Percentage Efficiency Increase (CAV vs Human) on the y-axis against Conflict Points on the x-axis.
    * Each point represents one trial.
    * Helps identify overall trends, the range of efficiency gains observed, and how the relative benefit of CAVs changes with intersection complexity.
This project is licensed under the MIT License.
