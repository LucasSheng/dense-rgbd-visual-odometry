function variance = computeResidualVariance(residual, nu)
% compute the variance of the residuals, assuming standard student-t
% distribution
%
% INPUT:
%   residual: a vector of size [num_pixels, 1]
%   nu: degrees of freedom nu
%
% OUTPUT:
%   variance: the scalar variance of the standard student-t distribution

variance = 1;
for iter = 1:10
    variance = mean(residual.^2 .* (nu + 1) ./ (nu + residual.^2./variance));
end
    
end