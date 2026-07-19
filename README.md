Project Overview:
This repository hosts a passive target localization and range estimation system. The main objective is to estimate a target's coordinates using self-position data and relative bearing angles. The implementation leverages MATLAB and Simulink to evaluate the performance of three state estimation filters: Recursive Least Squares (RLS), Extended Kalman Filter (EKF), and Unscented Kalman Filter (UKF), under simulated noise conditions as well as real-world sensor logs.

Project Components:
The project contains the following components:
- Raw Sensor Logs (.mat files): Physical coordinates and gyroscope data logged via MATLAB Mobile:
  - sensorlog_try31_ship.mat: Static target with a circular observer trajectory.
  - sensorlog_try62_ship.mat: Static target with a spiral observer trajectory.
  - sensorlog_try43_ship.mat and sensorlog_try43_target.mat: Dynamic scenario featuring a linear target and a lemniscate (figure-eight) observer trajectory.
- Preprocessing MATLAB Scripts (.m files):
  - Static_try31.m: Converts and processes raw sensor data for the static target / circular observer scenario (try31) into local metric coordinates.
  - Static_try62.m: Converts and processes raw sensor data for the static target / spiral observer scenario (try62) into local metric coordinates.
  - Dynamic.m: Generates simulated parameters or sets up the workspace for the linear target / lemniscate observer scenario (try43).
- Simulink Estimation Models (.mdl files): 
  - First_Model_2026_SIMULATION_ONLY_FINAL.mdl handles pure simulation runs under ideal conditions.
  - Second_Model_2026_Static_Target_Circ_Ship_WET.mdl processes real-world sensor data for static target tracking.
  - Third_Model_2026_Dynamic_Target_Circ_Ship_WET.mdl processes real-world sensor data for dynamic target tracking.

How to Use:
Save all repository files, including the .mat sensor logs, into your MATLAB working directory. Do not rename the files, as the scripts rely on their original names.

Scenario 1: Circular Observer Tracking a Static Target (try31)
1. Preprocess Data: Run the MATLAB script Static_try31.m first. This loads sensorlog_try31_ship.mat, performs coordinate transformations, and populates the workspace with the preprocessed timetable.
2. Run Range Estimation: Open the Simulink model Second_Model_2026_Static_Target_Circ_Ship_WET.mdl and click Run to process the preprocessed timetable and execute target range estimation.

Scenario 2: Spiral Observer Tracking a Static Target (try62)
1. Preprocess Data: Run the MATLAB script Static_try62.m first. This loads sensorlog_try62_ship.mat, performs coordinate transformations, and populates the workspace with the preprocessed timetable.
2. Run Range Estimation: Open the Simulink model Second_Model_2026_Static_Target_Circ_Ship_WET.mdl and click Run to process the preprocessed timetable and execute target range estimation.

Scenario 3: Lemniscate Observer Tracking a Linear Target (try43)
1. Preprocess Data: Run the MATLAB script Dynamic.m first to generate the trajectory configurations and set up workspace parameters for the moving target and observer.
2. Run Range Estimation: Open the Simulink model Third_Model_2026_Dynamic_Target_Circ_Ship_WET.mdl and click Run to execute the estimators.

Pure Simulation Scenario
1. Open the Simulink model First_Model_2026_SIMULATION_ONLY_FINAL.mdl and click Run to test the algorithms under pure simulation (ideal kinematics and synthetic Gaussian noise). Set the switches to select desired movement modes.

Requirements:
- MATLAB (with Simulink and Signal Processing Toolbox).
- Provided sensor data and script files in the same working directory.
