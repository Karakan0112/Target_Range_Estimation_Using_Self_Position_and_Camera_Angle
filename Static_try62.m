% Target Position Estimation - Data Preprocessing (10 Hz)

clear variables;
close all;
clc;

% 0. File Loading
file_name = 'sensorlog_try62_ship'; 
load([file_name, '.mat']); 

% 1. Time Alignment
t_start = max(Position.Timestamp(1), AngularVelocity.Timestamp(1));
t_end_natural = min(Position.Timestamp(end), AngularVelocity.Timestamp(end));

forced_duration = seconds(30);
if (t_end_natural - t_start) > forced_duration
    t_end = t_start + forced_duration;
else
    t_end = t_end_natural;
end

t_master = t_start : seconds(0.1) : t_end;
exp_time_10hz = seconds(t_master - t_master(1))';

% 2. Gyroscope Integration
yaw_rate_raw = AngularVelocity.Y; 
yaw_rate_10hz = interp1(AngularVelocity.Timestamp, yaw_rate_raw, t_master, 'linear', 'extrap')';

static_bias = mean(yaw_rate_10hz(1:30), 'omitnan'); 
yaw_rate_corrected = yaw_rate_10hz - static_bias;
relative_angle_change = cumtrapz(exp_time_10hz, yaw_rate_corrected); 

% 3. GPS Position Extraction & Smoothing (ENU)
ship_lat = Position.latitude;
ship_lon = Position.longitude;
ship_alt = Position.altitude;
wgs84 = wgs84Ellipsoid("meter");

[X_e, Y_e, Z_e] = geodetic2ecef(wgs84, Position.latitude, Position.longitude, Position.altitude); 
[X_enu_1hz, Y_enu_1hz, ~] = ecef2enu(X_e, Y_e, Z_e, Position.latitude(1), Position.longitude(1), Position.altitude(1), wgs84);

X_raw_10hz = interp1(Position.Timestamp, X_enu_1hz, t_master, 'linear', 'extrap')';
Y_raw_10hz = interp1(Position.Timestamp, Y_enu_1hz, t_master, 'linear', 'extrap')';

window_size = 15;
X = movmean(X_raw_10hz, window_size);
Y = movmean(Y_raw_10hz, window_size);

% 4. Target Initialization & Position Shift
target_lat = 32.097677;
target_long = 34.790020;
target_alt = 0; 

[target_X_e, target_Y_e, target_Z_e] = geodetic2ecef(wgs84, target_lat, target_long, target_alt);
[target_X, target_Y, target_Z] = ecef2enu(target_X_e, target_Y_e, target_Z_e, ship_lat(1), ship_lon(1), ship_alt(1), wgs84);

shift_amount = 70;
X = X + shift_amount;
Y = Y + shift_amount;
target_X = target_X + shift_amount;
target_Y = target_Y + shift_amount;

target_dist = sqrt((target_X - X(1))^2 + (target_Y - Y(1))^2);

% 5. Initial Bearing Calculation
beta_0 = atan2(target_Y - Y(1), target_X - X(1));  
bias_offset_rad = deg2rad(150); 
scale_factor = 0.8;
beta_measured = (beta_0 - bias_offset_rad) - (relative_angle_change * scale_factor);
beta_measured_wrapped = wrapToPi(beta_measured);
delay_sec = 2.8;
shift_idx = round(delay_sec / 0.1);
beta_measured_wrapped = [beta_measured_wrapped(shift_idx+1:end); repmat(beta_measured_wrapped(end), shift_idx, 1)];

% 6. Formatting for Simulink
numRepeats = 1000; 

X_vals = repmat(X, numRepeats, 1);
Y_vals = repmat(Y, numRepeats, 1);
bearingVals = repmat(beta_measured_wrapped, numRepeats, 1);

dt = seconds(0.1); 
newTime = (0:length(X_vals)-1)' * dt;

TT_X_inf = timetable(newTime, X_vals, 'VariableNames', {'X'});
TT_Y_inf = timetable(newTime, Y_vals, 'VariableNames', {'Y'});
TT_Bearing_Inf_Scaled = timetable(newTime, bearingVals, 'VariableNames', {'Relative_Bearing'});

% 7. Visualization Plotting
figure;
plot(X, Y, 'LineWidth', 2);
hold on;
plot(X_raw_10hz + shift_amount, Y_raw_10hz + shift_amount, 'k:', 'Color', [0.7 0.7 0.7]); 
scatter(X(1), Y(1), 100, 'g', 'filled');       
scatter(X(end), Y(end), 100, 'r', 'filled');   
scatter(target_X, target_Y, 300, 'p', 'MarkerEdgeColor', [0.2 0.6 0.9], 'MarkerFaceColor', 'b', 'LineWidth', 1.5); 
title('Smoothed Trajectory vs Target in ENU Coordinates');
xlabel('East (meters)');
ylabel('North (meters)');
legend('Smoothed Path', 'Raw GPS Data', 'Start', 'End', 'Target', 'FontSize', 12, 'Location', 'best');
grid on;
axis equal; 
hold off;

figure;
plot(TT_Bearing_Inf_Scaled.newTime(1:length(beta_measured)), rad2deg(beta_measured_wrapped));
title('Calculated Bearing to Target (Wrapped)');
xlabel('Time');
ylabel('Bearing (deg)');
grid on;

disp('--- Setup Complete ---');
disp(['Target X: ', num2str(target_X), ' m']);
disp(['Target Y: ', num2str(target_Y), ' m']);
disp(['Initial Distance: ', num2str(target_dist), ' m']);
disp(['Theoretical Initial Angle (beta_0): ', num2str(rad2deg(beta_0)), ' deg']);
disp('Data formatting complete. Ready for Simulink.');