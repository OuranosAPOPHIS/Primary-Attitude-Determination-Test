%% IMU Validation

% Import Calibration data. 

load('IMU_Calibration_Data')

% Import Log Files from Validation Test.

test_data = csvread('GSDataLog.csv',3,5);

%% Post-Processing of Accel and Gryo Measurements

A_xyz_v = test_data(:,1:3);
% measured accelerations during validation test

W_xyz_v = test_data(:,4:6);
% measured angular velocities during validation test

Paccel = asind(A_xyz_v(:,1));
% estimated pitch from accelerometer reading

Raccel = atan2d(A_xyz_v(:,2),A_xyz_v(:,3));
% estimated roll from accelerometer reading

