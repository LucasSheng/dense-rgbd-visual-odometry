function warped_pointclouds = warpPointCloud(pointclouds, pose)
% warp the point cloud in the 3D space
%
% INPUT:
%   pointclouds: a matrix of size [num_points, 3]
%   pose: a matrix of size [4, 4]
%
% OUTPUT:
%   warped_pointclouds: a matrix of size [num_points, 3]

warped_pointclouds = bsxfun(@plus, pointclouds * pose(1:3, 1:3)', pose(1:3, 4)');
end