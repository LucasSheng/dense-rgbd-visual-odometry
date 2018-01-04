function img_pyr = constructPyramid(img, num_levels)
% construct image pyramid for the input image
%
% INPUT:
%   img: a grayscale image
%   num_levels: number of pyramid levels
%
% OUTPUT:
%   img_pyr: a cell of image pyramid

if size(img, 3) > 1
    img = rgb2gray(img);
end

img_pyr = cell(num_levels, 1);
img_pyr{1} = img;
for i = 2:num_levels
    img = imresize(img, 0.5, 'nearest');
    img_pyr{i} = img;
end

end