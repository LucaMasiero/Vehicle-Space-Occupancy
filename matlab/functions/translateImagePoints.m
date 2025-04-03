function A = translateImagePoints(A, p)
% translateImagePoints Translate 2D image points stored in the Nx2 matrix A from
% the "current" reference frame (centered in the top-left corner of the
% image, with x going to the right and y going downward) to the reference frame
% centered in the principal point p(px,py), where the x-axis goes to the left
% and the y-axis goes upward (from the camera point of view).
%
% Inputs:
%   A - Nx2 matrix where each row represents the (x,y) coordinates of an
%       image point in the reference frame centered in the upper-left corner
%       of the image
%   p - 1x2 vector containing x and y coordinates of the principal point w.r.t. top-left corner of image
% 
% Output:
%   A - Nx2 matrix containing translated points
% 

R_mirror = [-1 0; 0 -1]; %180 degree rotation
T = R_mirror*p'; %translate to center in p

for i=1:size(A,1)
    A(i,:) = (R_mirror*A(i,:)' - T)'; %apply transformation to each point
end 

end