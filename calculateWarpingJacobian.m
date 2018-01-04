function jacobian = calculateWarpingJacobian(warped_pointclouds, pointclouds, pose, K)
% calculate the warping jacobian matrix
%
% INPUT:
%   warped_pointclouds: a matrix of size [num_pixels, 3]
%   pointclouds: a matrix of size [num_pixels, 3]
%   pose: the pose for the warping
%   K: the intrinsic matrix of the camera
%
% OUTPUT:
%   jacobian: jacobian matrix of the warping operation

fx = K(1, 1);
fy = K(2, 2);

num_points = size(pointclouds, 1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% step-1: projection gradient
% pointcloud after warping
warped_x = warped_pointclouds(:, 1);
warped_y = warped_pointclouds(:, 2);
warped_z = warped_pointclouds(:, 3);

proj_grad = cat(1, ...
    [fx./warped_z, zeros(num_points, 1), -fx.*warped_x./warped_z.^2], ...
    [zeros(num_points, 1), fy./warped_z, -fy.*warped_y./warped_z.^2]); 
proj_grad = reshape(proj_grad, [], 2, 3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% step-2: rigid point-transformation gradient
% pointcloud before warping
x = pointclouds(:, 1);
y = pointclouds(:, 2);
z = pointclouds(:, 3);

rot_grad = kron(eye(3), cat(2, x, y, z));
rot_grad = reshape(rot_grad, [], 3, 9);
trans_grad = kron(eye(3), ones(num_points, 1));
trans_grad = reshape(trans_grad, [], 3, 3);
rigid_trans_grad = cat(3, rot_grad, trans_grad);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% step-3: twist motion gradient
twist_rot_grad = cat(1, ...
    [zeros(3, 1), pose(1:3, 3), -pose(1:3, 2), zeros(3, 3)], ...
    [-pose(1:3, 3), zeros(3, 1), pose(1:3, 1), zeros(3, 3)], ...
    [pose(1:3, 2), -pose(1:3, 1), zeros(3, 1), zeros(3, 3)]);
twist_trans_grad = cat(1, ...
    [0, pose(3, 4), -pose(2, 4), 1, 0, 0], ...
    [-pose(3, 4), 0, pose(1, 4), 0, 1, 0], ...
    [pose(2, 4), -pose(1, 4), 0, 0, 0, 1]);
twist_grad = cat(1, twist_rot_grad, twist_trans_grad);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% calculate the chained gradient
chained_gradient = zeros(num_points, 2, 12);
for i = 1:2
    for j = 1:12
        proj_grad_temp = squeeze(proj_grad(:, i, :));
        rigid_trans_grad_temp = squeeze(rigid_trans_grad(:, :, j));
        chained_gradient(:, i, j) = dot(proj_grad_temp, rigid_trans_grad_temp, 2);
    end
end
% multiply the twist gradient
chained_gradient = reshape(chained_gradient, [], 12) * twist_grad;
jacobian = reshape(chained_gradient, [], 2, 6);

end