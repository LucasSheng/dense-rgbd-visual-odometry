function normalized_coords = constructNormalizedCoordinate(height, width, K)
% Construct normalized coordinate
%
% INPUT:
%   height: the height of the input image
%   width: the width of the input image
%   K: intrinsic parameters of the camera
%
% OUTPUT:
%   normalized_coords: as a 2-channel image

fx = K(1, 1);
fy = K(2, 2);
u0 = K(1, 3);
v0 = K(2, 3);

[tilde_x, tilde_y] = meshgrid(1:width, 1:height);
tilde_x = (tilde_x - u0)/fx;
tilde_y = (tilde_y - v0)/fy;

normalized_coords = cat(3, tilde_x, tilde_y);

end