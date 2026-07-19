% =========================================================================
% Target Position Estimation - Data Preprocessing (10 Hz)
% =========================================================================

file_name = 'sensorlog_try31_ship'; % Base name of the .mat file
load([file_name, '.mat']); % Load the sensor data

% -------------------------------------------------------------------------
% 1. True 10 Hz Time Alignment & Extraction
% -------------------------------------------------------------------------
% Create a synthetic 10 Hz master clock from the very first to very last timestamp
t_start = max(Position.Timestamp(1), AngularVelocity.Timestamp(1));
t_end_natural   = min(Position.Timestamp(end), AngularVelocity.Timestamp(end));

% Force the dataset to consider only the first 40 seconds (if the recording is longer)
forced_duration = seconds(30);
if (t_end_natural - t_start) > forced_duration
t_end = t_start + forced_duration;
else
t_end = t_end_natural;
end

t_master = t_start : seconds(0.1) : t_end;

% Compute purely numeric elapsed time (in seconds) for math integration
exp_time_10hz = seconds(t_master - t_master(1))';

% -------------------------------------------------------------------------
% 2. Gyroscope Integration (Correct Axis)
% -------------------------------------------------------------------------
% CRITICAL: Use Y for Portrait mount, use X for Landscape mount!
yaw_rate_raw = AngularVelocity.Y; 

% Interpolate to our 10 Hz master clock
yaw_rate_10hz = interp1(AngularVelocity.Timestamp, yaw_rate_raw, t_master, 'linear', 'extrap')';

% Calculate bias from the first 30 samples (3 seconds of standing still)
static_bias = mean(yaw_rate_10hz(1:30), 'omitnan'); 
yaw_rate_corrected = yaw_rate_10hz - static_bias;

% Integrate the correct axis
relative_angle_change = cumtrapz(exp_time_10hz, yaw_rate_corrected); 

% -------------------------------------------------------------------------
% 3. GPS Position Extraction & Smoothing (1 Hz -> 10 Hz ENU)
% -------------------------------------------------------------------------
ship_lat = Position.latitude;
ship_lon = Position.longitude;
ship_alt = Position.altitude;

wgs84 = wgs84Ellipsoid("meter");

% Convert original 1 Hz geodetic to ENU first
[X_e, Y_e, Z_e] = geodetic2ecef(wgs84, Position.latitude, Position.longitude, Position.altitude); 
[X_enu_1hz, Y_enu_1hz, ~] = ecef2enu(X_e, Y_e, Z_e, Position.latitude(1), Position.longitude(1), Position.altitude(1), wgs84);

% Interpolate the ENU coordinates up to our smooth 10 Hz master clock
X_raw_10hz = interp1(Position.Timestamp, X_enu_1hz, t_master, 'linear', 'extrap')';
Y_raw_10hz = interp1(Position.Timestamp, Y_enu_1hz, t_master, 'linear', 'extrap')';

% Now apply the 15-sample Moving Average Filter (1.5 seconds of data)
window_size = 15;
X = movmean(X_raw_10hz, window_size);
Y = movmean(Y_raw_10hz, window_size);

% -------------------------------------------------------------------------
% 4. Target Initialization
% -------------------------------------------------------------------------
% Static target's location
target_lat = 32.097714;
target_long = 34.789853;
target_alt = 0; 

% Convert Target to ENU relative to the initial ship position
[target_X_e, target_Y_e, target_Z_e] = geodetic2ecef(wgs84, target_lat, target_long, target_alt);
[target_X, target_Y, target_Z] = ecef2enu(target_X_e, target_Y_e, target_Z_e, ship_lat(1), ship_lon(1), ship_alt(1), wgs84);

% --- APPLY THE COORDINATE SHIFT HERE ---
% Shifting both the observer's path and the target's position by 70 meters
shift_amount = 70;
X = X + shift_amount;
Y = Y + shift_amount;
target_X = target_X + shift_amount;
target_Y = target_Y + shift_amount;
% ---------------------------------------

% Calculate distance between Target and Observer start point
target_dist = sqrt((target_X - X(1))^2 + (target_Y - Y(1))^2); % Horizontal distance

% -------------------------------------------------------------------------
% 5. True Initial Bearing Calculation
% -------------------------------------------------------------------------
% Calculate true initial theoretical angle using smoothed start pos (0,0)
beta_0 = atan2(target_Y - Y(1), target_X - X(1));  

% Add initial theoretical angle to integrated relative angle
beta_measured = beta_0 + relative_angle_change;

% Wrap angle between -pi and pi to keep EKF/UKF math clean
beta_measured_wrapped = wrapToPi(beta_measured);

% -------------------------------------------------------------------------
% 6. Formatting for Simulink (Infinite Series at 10 Hz)
% -------------------------------------------------------------------------
numRepeats = 1000; % Replicate to create a long simulation run

% Replicate data
X_vals = repmat(X, numRepeats, 1);
Y_vals = repmat(Y, numRepeats, 1);
bearingVals = repmat(beta_measured_wrapped, numRepeats, 1);

% Create a new continuous time vector at 10 Hz (dt = 0.1)
dt = seconds(0.1); 
newTime = (0:length(X_vals)-1)' * dt;

% Create the final timetables exactly as Simulink expects them
TT_X_inf = timetable(newTime, X_vals, 'VariableNames', {'X'});
TT_Y_inf = timetable(newTime, Y_vals, 'VariableNames', {'Y'});
TT_Bearing_Inf_Scaled = timetable(newTime, bearingVals, 'VariableNames', {'Relative_Bearing'});

% -------------------------------------------------------------------------
% 7. Visualization Plotting
% -------------------------------------------------------------------------
% Plot ship's path in local ENU frame with target and endpoints
figure
plot(X, Y, 'LineWidth', 2);
hold on;
plot(X_raw_10hz + shift_amount, Y_raw_10hz + shift_amount, 'k:', 'Color', [0.7 0.7 0.7]); % Plot raw noisy GPS behind it
scatter(X(1), Y(1), 100, 'g', 'filled');       % Start position
scatter(X(end), Y(end), 100, 'r', 'filled');   % End position
scatter(target_X, target_Y, 300, 'p', 'MarkerEdgeColor', [0.2 0.6 0.9], 'MarkerFaceColor', 'b', 'LineWidth', 1.5); 
title('Smoothed Trajectory vs Target in ENU Coordinates');
xlabel('East (meters)');
ylabel('North (meters)');
legend('Smoothed Path', 'Raw GPS Data', 'Start', 'End', 'Target', 'FontSize', 12, 'Location', 'best');
grid on;
axis equal; % Ensures the aspect ratio is 1:1 for accurate spatial viewing
hold off;

% Plot time-series of measured bearing
figure
plot(TT_Bearing_Inf_Scaled.newTime(1:length(beta_measured)), rad2deg(beta_measured_wrapped));
title('Calculated Bearing to Target (Wrapped)');
xlabel('Time');
ylabel('Bearing (deg)');
grid on;

% Print Console Output
disp('--- Setup Complete ---');
disp(['Target X: ', num2str(target_X), ' m']);
disp(['Target Y: ', num2str(target_Y), ' m']);
disp(['Initial Distance: ', num2str(target_dist), ' m']);
disp(['Theoretical Initial Angle (beta_0): ', num2str(rad2deg(beta_0)), ' deg']);
disp('Data formatting complete. Ready for Simulink.');