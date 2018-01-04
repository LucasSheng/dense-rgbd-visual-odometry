% example folder
exp_folder = 'examples';

% Kinect intrinsic parameters
K = dlmread(fullfile(exp_folder, 'kinect_params.txt'));

% load (image, depth) pairs and ground truth relative pose
case_folder = 'case_2';

%% load previous image and depth pairs
img_prev = im2double(imread(fullfile(exp_folder, case_folder, 'img_prev.png')));
dep_prev = double(imread(fullfile(exp_folder, case_folder, 'dep_prev.png')));

% load current image
img_curr = im2double(imread(fullfile(exp_folder, case_folder, 'img_curr.png')));
dep_curr = double(imread(fullfile(exp_folder, case_folder, 'dep_curr.png')));

% convert the image and depth data
img_prev = rgb2gray(img_prev); img_curr = rgb2gray(img_curr);
dep_prev = dep_prev/5000; dep_curr = dep_curr/5000;

% load the ground-truth poses for two frames: pose_vec --> [tx, ty, tz, wx, wy, wz, wq]
gt_pose_prev = dlmread(fullfile(exp_folder, case_folder, 'pose_prev.txt'));
gt_pose_curr = dlmread(fullfile(exp_folder, case_folder, 'pose_curr.txt'));

gt_pose_prev = convertMotionVectorToMatrix(gt_pose_prev);
gt_pose_curr = convertMotionVectorToMatrix(gt_pose_curr);

% calculate the relative pose
gt_pose_rel = getRelativePose(gt_pose_curr, gt_pose_prev);

%% warping by the ground-truth relative pose
[warped_image, valid_mask] = warpImage(img_curr, dep_prev, gt_pose_rel, K);
error = mean((warped_image(valid_mask) - img_prev(valid_mask)).^2);
disp(error);

figure(3);
imshow(abs(warped_image - img_prev) .* valid_mask, [])

%% calculate the relative pose from two RGB-D frames
num_levels = 5;
isVerbose = true;

[pose_rel, score] = estimateVisualOdometry(img_curr, img_prev, dep_prev, K, num_levels, isVerbose);