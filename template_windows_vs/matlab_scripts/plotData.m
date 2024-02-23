
clc; clear; close all;

base_folder = "C:\Users\larrydong\Desktop\chrono_speed5_step0.2\";

%% Draw the IMU data;

% Define the CSV file name
filename = base_folder + "imu.csv";

% Read the CSV file, skipping the first row (header)
opts = delimitedTextImportOptions('NumVariables', 7, ...
                                  'DataLines', 2, ...
                                  'Delimiter', ',');
opts.VariableNames = ["ts", "ax", "ay", "az", "wx", "wy", "wz"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double"];
imuData = readtable(filename, opts);

% Extracting data for plotting
ts = imuData.ts;
ax = imuData.ax;
ay = imuData.ay;
az = imuData.az;
wx = imuData.wx;
wy = imuData.wy;
wz = imuData.wz;

% Plotting
figure;

% Acceleration subplot
subplot(2,1,1);
plot(ts, ax, 'r', ts, ay, 'g', ts, az, 'b');
title('Acceleration Data');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
legend('ax', 'ay', 'az');

% Gyroscope subplot
subplot(2,1,2);
plot(ts, wx, 'r', ts, wy, 'g', ts, wz, 'b');
title('Gyroscope Data');
xlabel('Time (s)');
ylabel('Angular velocity (rad/s)');
legend('wx', 'wy', 'wz');


%% Draw the trajectory
% Load ground truth data from CSV
filename = base_folder + "gt.csv";
data = readtable(filename, 'Format', '%f%f%f%f%f%f%f%f', 'HeaderLines', 1);

% Extract data
ts = data{:, 1};
pos_x = data{:, 2};
pos_y = data{:, 3};
pos_z = data{:, 4};
qw = data{:, 5};
qx = data{:, 6};
qy = data{:, 7};
qz = data{:, 8};

% Convert quaternions to Euler angles
eul = quat2eul([qw qx qy qz]);

% Plot position and orientation in subfigures
figure;
subplot(2, 1, 1);
plot(ts, pos_x, 'r', ts, pos_y, 'g', ts, pos_z, 'b');
title('Position');
xlabel('Time (s)');
ylabel('Position (m)');
legend('X', 'Y', 'Z');

subplot(2, 1, 2);
% plot(ts, eul);
plot(ts, eul(:,1), 'r', ts, eul(:,2), 'g', ts, eul(:,3), 'b');
title('Orientation (Euler angles)');
xlabel('Time (s)');
ylabel('Angle (rad)');
legend('Roll', 'Pitch', 'Yaw');

% Draw a 2D graph of the vehicle trajectory
figure;
plot(pos_x, pos_y);
title('2D Vehicle Trajectory');
xlabel('Position X (m)');
ylabel('Position Y (m)');
grid on;
axis equal;
