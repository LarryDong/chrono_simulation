clc;clear;close all;

% define terrin size.
resolution = 100;
% resolution = round(width / min_scale);

output_folder = "C:\Users\larrydong\Desktop\";

%% create one step terrain
img = zeros(resolution, resolution, 1);
region_row = 40:60;
region_col = 1:20;
img(region_row, region_col) = ones(length(region_row), length(region_col));
img = img * 255;
imwrite(img, output_folder+"step1.bmp");

