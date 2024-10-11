clc;clear;close all;

% Read the data from 'a.csv'
data = readtable('/home/larry/data/chrono/smallA-Gyro-2024.10.11/gt.csv');

% Extract the specified columns: 
% 'Timestamp', 'IMU_pos_x', 'IMU_pos_y', 'IMU_pos_z', 'IMU_qx', 'IMU_qy', 'IMU_qz', 'IMU_qw'
outputData = data(:, 1:8);

% Write the extracted data to 'output.csv' with space-separated values
writetable(outputData, 'output.csv', 'Delimiter', ' ', 'WriteVariableNames', false);
