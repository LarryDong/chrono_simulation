% Read control points from input.txt
data = dlmread('control_points.txt');

% Extract x and y coordinates from the data
x = data(:,1);
y = data(:,2);

% Calculate cumulative distance along the curve
cumulative_distance = [0; cumsum(sqrt(diff(x).^2 + diff(y).^2))];

% Interpolate points along the curve with approximately 1 meter spacing
desired_spacing = 1; % meters
t_interp = interp1(cumulative_distance, linspace(0, 1, numel(x)), 0:desired_spacing:max(cumulative_distance));

% Generate spline trajectory
px = spline(1:numel(x), x, t_interp);
py = spline(1:numel(y), y, t_interp);

% Write trajectory to output.txt
output_data = [px; py]';
dlmwrite('output.txt', output_data, 'precision', 6);
