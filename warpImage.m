function [warped_image, valid_mask] = warpImage(img_curr, dep_prev, pose_rel, K)
% warp the current image to the previous image coordinate
%
% INPUT:
%   img_curr: current grayscale image
%   dep_prev: previous depth image
%   pose_rel: relative pose between the current and previous frame
%   K: intrinsic camera parameters
%
% OUTPUT:
%   warped_image: warped image by the rigid transformation
%   valid_mask: valid pixels in the warped image

[height, width] = size(img_curr);

[pointclouds, valid_mask] = reprojectDepthImage(dep_prev, K);
warped_pointclouds = warpPointCloud(pointclouds, pose_rel);
[warped_img_coords, warped_valid_mask] = projectPointCloud(warped_pointclouds, K, height, width);

warped_intensities = interp2(img_curr, warped_img_coords(:, 1), warped_img_coords(:, 2), 'linear', 0);

valid_indices = find(valid_mask);
valid_indices = valid_indices(warped_valid_mask);
valid_mask = zeros(height, width);
valid_mask(valid_indices) = 1;
valid_mask = logical(valid_mask);

warped_image = zeros(height, width);
warped_image(valid_indices) = warped_intensities;

end