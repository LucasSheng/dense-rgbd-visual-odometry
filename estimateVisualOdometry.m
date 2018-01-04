function [pose_rel, score] = estimateVisualOdometry(img_curr, img_prev, dep_prev, K, num_levels, isVerbose)
% estimate visual odometry from two adjacent frames
%
% INPUT:
%   img_curr: current RGB frame
%   img_prev: previous RGB frame
%   dep_prev: previous depth frame
%   K: intrinsic parameters of the camera
%   num_levels: pyramid levels for the estimation
%   isVerbose: whether to show intermediate results
%
% OUTPUT:
%   pose_rel: relative pose
%   score: fitting score

% construct image pyramids
img_curr_pyr = constructPyramid(img_curr, num_levels);
img_prev_pyr = constructPyramid(img_prev, num_levels);

% construct depth pyramid for the previous frame
dep_prev_pyr = constructPyramid(dep_prev, num_levels);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% estimate the relative pose from coarse to fine scales

% initialize the relative pose and its increment
pose_rel = eye(4);
increment = zeros(6, 1);

% convert the vectorize motion increment to update the relative pose
increment = twistexp(increment);
pose_rel = increment * pose_rel;

% modify the camera parameters to fit each pyramid
K_pyr = K;
K_pyr(1:2, :) = K_pyr(1:2, :) / (2^(num_levels-1));

for n = num_levels:-1:1
    
    % image size
    [height, width] = size(dep_prev_pyr{n});
    
    % get valid point clouds in the previous frame
    [pointclouds, valid_mask] = reprojectDepthImage(dep_prev_pyr{n}, K_pyr);
    
    % warp pointclouds and prune invalid points
    warped_pointclouds = warpPointCloud(pointclouds, pose_rel);
    [warped_img_coordinates, valid_points] = projectPointCloud(warped_pointclouds, K_pyr, height, width);
    
    warped_pointclouds = warped_pointclouds(valid_points, :);
    pointclouds = pointclouds(valid_points, :);
    
    % spatial gradient in the current frame
    [Gx_curr, Gy_curr] = imgradientxy(img_curr_pyr{n}, 'CentralDifference');
    Gx_curr = interp2(Gx_curr, warped_img_coordinates(:, 1), warped_img_coordinates(:, 2), 'linear', 0);
    Gy_curr = interp2(Gy_curr, warped_img_coordinates(:, 1), warped_img_coordinates(:, 2), 'linear', 0);
    Gs_curr = cat(2, Gx_curr, Gy_curr);

    % temporal visual difference
    Gt_prev = img_prev_pyr{n};
    Gt_prev = Gt_prev(valid_mask);
    Gt_prev = Gt_prev(valid_points);
    Gt_curr = img_curr_pyr{n};
    Gt_curr = interp2(Gt_curr, warped_img_coordinates(:, 1), warped_img_coordinates(:, 2), 'linear', 0);
    Gt = Gt_curr - Gt_prev;

    % calculate the warping Jacobian
    warp_jacobian = calculateWarpingJacobian(warped_pointclouds, pointclouds, pose_rel, K_pyr);

    % calculate the compositive jacobian
    comp_jacobian = squeeze(sum(bsxfun(@times, Gs_curr, warp_jacobian), 2));

%     % adjust the importance of each residual elements, when required
%     % necessary pruning of bad correspondences
%     % compute the weight
%     variance = computeResidualVariance(Gt, 5);
%     weights = sqrt((5 + 1) ./ (5 + Gt.^2./variance));
%     comp_jacobian = bsxfun(@times, comp_jacobian, weights);
%     Gt = Gt.*weights;
    
    % calculate the increment motion
    increment = -(comp_jacobian'*comp_jacobian)\(comp_jacobian'*Gt);

    % get the current relative pose
    increment = twistexp(increment);
    pose_rel = increment * pose_rel;

    % intermediate results
    if isVerbose
        [warped_image, valid_mask] = warpImage(img_curr, dep_prev, pose_rel, K);
        error = mean((warped_image(valid_mask) - img_prev(valid_mask)).^2);
        disp(['visual consistency score in level ' num2str(n) ' is ' num2str(error)]);
        
        figure(1);
        imshow(abs(warped_image - img_prev).*valid_mask, [])
    end
    
    % increse the focal length
    K_pyr(1:2, :) = K_pyr(1:2, :) * 2;
end

% get the final score
[warped_image, valid_mask] = warpImage(img_curr, dep_prev, pose_rel, K);
score = mean((warped_image(valid_mask) - img_prev(valid_mask)).^2);

if isVerbose
    disp(['The fitting score is ' num2str(error)]);
end

end
