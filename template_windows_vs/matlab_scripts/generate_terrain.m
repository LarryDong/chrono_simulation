
terrain_long = 80;
terrain_width = 10;

spacial_resolution = 1;		% 1m resolution

image_row = terrain_long / spacial_resolution;
image_col = terrain_width / spacial_resolution;


z = zeros(image_row, image_col);
z(2:image_row-1, 2:image_col-1) = rand(image_row-2, image_col-2);

z = uint8(z*255);
imwrite(z, "terrain.bmp")