function points = selectFeatures(image_path)
% SELECTFOURPOINTS - Allows user to select 4 points from an image
%
% Syntax:  points = selectFeatures(image_path)
%
% Inputs:
%    image_path - String containing the path to the image file
%
% Outputs:
%    points - 2x4 matrix where each column contains [x; y] coordinates of a point
%
% Example:
%    points = selectFourPoints('myimage.jpg');
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none

% Read and display the image
img = imread(image_path);
figure;
imshow(img);
hold on;

% Set up the title with instructions
title('Click on 4 points in the image. Press Enter when done.');

% Initialize the output matrix
points = zeros(2, 4);

% Prompt user to select 4 points
disp('Please select 4 points on the image...');

% Get the 4 points from the user
for i = 1:4
    % Wait for the user to click
    [x, y] = ginput(1);
    
    % Store the coordinates
    points(1, i) = x;
    points(2, i) = y;
    
    % Plot the selected point with a marker
    plot(x, y, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    
    % Display the point number
    text(x+10, y+10, num2str(i), 'Color', 'red', 'FontSize', 12, 'FontWeight', 'bold');
    
    % Inform the user
    disp(['Point ' num2str(i) ' selected: (' num2str(x) ', ' num2str(y) ')']);
end

% Close the figure if needed (uncomment if you want the figure to close automatically)
% close;

% Display the result
disp('Points selection complete. Result matrix:');
disp(points);

end