function pose_rel = getRelativePose(pose_curr, pose_prev)
% get relative pose from the pose parameters between adjacent frames
%
% INPUT:
%   pose_curr: a [4, 4] pose matrix from the current frame
%   pose_prev: a [4, 4] pose matrix from the previous frame
%
% OUTPUT:
%   pose_rel: a [4, 4] relative pose matrix from the previous frame to the
%   current frame

pose_rel = pose_curr \ pose_prev;

end