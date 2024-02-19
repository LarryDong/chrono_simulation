
clc; clear; close all;

% Parameters
radius = 45; % radius of the circle
d = 1; % distance between adjacent control points, default is 1m
center = [0, 0]; % center of the circle

% Calculate the number of points
circumference = 2 * pi * radius;
N = round(circumference / d);

% Generate points
theta = linspace(0, 2*pi, N+1); % +1 to make the start and end point the same
theta(end) = []; % remove the duplicate end point to maintain exact distance d between points
x = center(1) + radius * cos(theta);
y = center(2) + radius * sin(theta);
z = zeros(size(x)); % assuming z is constant at 0

% Write to a text file
filename = 'trajectory_control_points.txt';
fileID = fopen(filename, 'w');
fprintf(fileID, '%d %d\n', N, 3);
for i = 1:length(x)
    fprintf(fileID, '%.6f %.6f %.6f\n', x(i), y(i), z(i));
end
fclose(fileID);