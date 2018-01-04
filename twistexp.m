function incre_pose = twistexp(increment)
% calculate the increment pose
%
% INPUT:
%   increment: a vector as [wx, wy, wz, tx, ty, tz]
%
% OUTPUT:
%   incre_pose: a [4, 4] matrix about the rigid transformation

wx = increment(1);
wy = increment(2);
wz = increment(3);

incre_pose = eye(4);
incre_pose(1:3, 4) = increment(4:6);
incre_pose(1, 2) = -wz;
incre_pose(1, 3) = wy;
incre_pose(2, 1) = wz;
incre_pose(2, 3) = -wx;
incre_pose(3, 1) = -wy;
incre_pose(3, 2) = wx;

end