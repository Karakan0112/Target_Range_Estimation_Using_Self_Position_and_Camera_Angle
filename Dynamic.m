% Passive Target Range Estimation - Dynamic Target Preprocessing (10 Hz)

clear variables;
close all;
clc;

% 0. File Loading
obs_file = 'sensorlog_try43_ship.mat';
tgt_file = 'sensorlog_try43_target.mat';

obs_data = load(obs_file);
tgt_data = load(tgt_file);

% 1. Time Alignment & Gyroscope Calibration
t_start_absolute = max([obs_data.Position.Timestamp(1), obs_data.AngularVelocity.Timestamp(1), tgt_data.Position.Timestamp(1)]);
t_end = min(obs_data.Position.Timestamp(end), obs_data.AngularVelocity.Timestamp(end));

yaw_rate_raw = obs_data.AngularVelocity.Y; 

t_calib = t_start_absolute : seconds(0.1) : (t_start_absolute + seconds(3));
yaw_rate_calib = interp1(obs_data.AngularVelocity.Timestamp, yaw_rate_raw, t_calib, 'linear', 'extrap');
static_bias = mean(yaw_rate_calib, 'omitnan');

t_start_processing = t_start_absolute + seconds(5);
t_master = t_start_processing : seconds(0.1) : t_end;
exp_time_10hz = seconds(t_master - t_master(1))';

% 2. Gyroscope Integration
yaw_rate_10hz = interp1(obs_data.AngularVelocity.Timestamp, yaw_rate_raw, t_master, 'linear', 'extrap')';
yaw_rate_corrected = yaw_rate_10hz - static_bias;
relative_angle_change = cumtrapz(exp_time_10hz, yaw_rate_corrected); 

% 3. GPS Position Extraction & Smoothing (ENU)
wgs84 = wgs84Ellipsoid("meter");

lat0 = obs_data.Position.latitude(1);
lon0 = obs_data.Position.longitude(1);
alt0 = obs_data.Position.altitude(1);

[X_e_obs, Y_e_obs, Z_e_obs] = geodetic2ecef(wgs84, obs_data.Position.latitude, obs_data.Position.longitude, obs_data.Position.altitude); 
[X_enu_obs_1hz, Y_enu_obs_1hz, ~] = ecef2enu(X_e_obs, Y_e_obs, Z_e_obs, lat0, lon0, alt0, wgs84);

[X_e_tgt, Y_e_tgt, Z_e_tgt] = geodetic2ecef(wgs84, tgt_data.Position.latitude, tgt_data.Position.longitude, tgt_data.Position.altitude); 
[X_enu_tgt_1hz, Y_enu_tgt_1hz, ~] = ecef2enu(X_e_tgt, Y_e_tgt, Z_e_tgt, lat0, lon0, alt0, wgs84);

X_obs_10hz = interp1(obs_data.Position.Timestamp, X_enu_obs_1hz, t_master, 'linear', 'extrap')';
Y_obs_10hz = interp1(obs_data.Position.Timestamp, Y_enu_obs_1hz, t_master, 'linear', 'extrap')';
X_tgt_10hz = interp1(tgt_data.Position.Timestamp, X_enu_tgt_1hz, t_master, 'linear', 'extrap')';
Y_tgt_10hz = interp1(tgt_data.Position.Timestamp, Y_enu_tgt_1hz, t_master, 'linear', 'extrap')';

window_size = 15;
X_obs = movmean(X_obs_10hz, window_size);
Y_obs = movmean(Y_obs_10hz, window_size);
X_tgt = movmean(X_tgt_10hz, window_size);
Y_tgt = movmean(Y_tgt_10hz, window_size);

shift_amount = 70;
X_obs = X_obs + shift_amount;
Y_obs = Y_obs + shift_amount;
X_tgt = X_tgt + shift_amount;
Y_tgt = Y_tgt + shift_amount;

% 4. Dynamic Ground Truth & Gyroscope Anchoring
beta_theoretical = atan2(Y_tgt - Y_obs, X_tgt - X_obs);
beta_0 = beta_theoretical(1);
beta_measured = beta_0 + relative_angle_change; 
beta_measured_wrapped = wrapToPi(beta_measured);

% 5. Formatting for Simulink
numRepeats = 1000; 

X_obs_vals = repmat(X_obs, numRepeats, 1);
Y_obs_vals = repmat(Y_obs, numRepeats, 1);
X_tgt_vals = repmat(X_tgt, numRepeats, 1);
Y_tgt_vals = repmat(Y_tgt, numRepeats, 1);
bearingVals = repmat(beta_measured_wrapped, numRepeats, 1);

dt = seconds(0.1); 
newTime = (0:length(X_obs)-1)' * dt;

TT_X_obs = timetable(newTime, X_obs, 'VariableNames', {'X_obs'});
TT_Y_obs = timetable(newTime, Y_obs, 'VariableNames', {'Y_obs'});
TT_X_tgt = timetable(newTime, X_tgt, 'VariableNames', {'X_tgt'});
TT_Y_tgt = timetable(newTime, Y_tgt, 'VariableNames', {'Y_tgt'});
TT_Bearing_Inf_Scaled = timetable(newTime, beta_measured_wrapped, 'VariableNames', {'Relative_Bearing'});

% 6. Visualization Plotting
figure;
plot(X_obs, Y_obs, 'b-', 'LineWidth', 2); hold on;
plot(X_tgt, Y_tgt, 'r-', 'LineWidth', 2);
scatter(X_obs(1), Y_obs(1), 100, 'b', 'filled'); 
scatter(X_tgt(1), Y_tgt(1), 100, 'r', 'filled'); 
title('Smoothed Trajectories in Local ENU System');
xlabel('East (meters)'); ylabel('North (meters)');
legend('Observer Path', 'Target Path', 'Observer Start', 'Target Start', 'Location', 'best');
grid on; axis equal; hold off;

figure;
plot(exp_time_10hz, rad2deg(wrapToPi(beta_theoretical)), 'y-', 'LineWidth', 1.5); hold on;
plot(exp_time_10hz, rad2deg(beta_measured_wrapped), 'b-', 'LineWidth', 1.5);
title('Dynamic Bearing: Theoretical vs Measured');
xlabel('Time (seconds)'); ylabel('Bearing (degrees)');
legend('Theoretical Angle', 'Measured Angle (Integrated)', 'Location', 'best');
grid on; hold off;

disp('Dynamic Target Setup Complete');
disp(['Static Bias: ', num2str(static_bias), ' rad/s']);
disp(['Initial Target Range: ', num2str(sqrt((X_tgt(1)-X_obs(1))^2 + (Y_tgt(1)-Y_obs(1))^2)), ' m']);
disp(['Initial Theoretical Angle (beta_0): ', num2str(rad2deg(beta_0)), ' deg']);