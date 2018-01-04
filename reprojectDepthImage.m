function [pointclouds, valid_mask] = reprojectDepthImage(depth, K)
% Reproject the depth image to 3D point clouds
%
% INPUT:
%   depth: a grayscale image
%   K: intrinsic parameters of the camera
%
% OUTPUT:
%   pointclouds: a 3-channel point list
%   valid_mask: a binary mask

height = size(depth, 1);
width = size(depth, 2);

valid_mask = depth > 0;

normalized_coords = constructNormalizedCoordinate(height, width, K);
pointclouds = cat(3, bsxfun(@times, normalized_coords, depth), depth);

% convert pointclouds to a list
pointclouds = reshape(pointclouds, [], 3);
pointclouds = pointclouds(valid_mask, :);

end