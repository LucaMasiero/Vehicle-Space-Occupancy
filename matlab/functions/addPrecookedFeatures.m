function [image_points, m] = addPrecookedFeatures()

    % Select sequence folder
    regstr = '.*\\([^\\]+)\\?$';  % to match the last folder-name
    dirPath = uigetdir(what('imgs\pandina\iPhone\').path);
    dirName = regexp(dirPath, regstr, 'tokens');
    dirName = dirName{1}{1};
   
    % Select frames
    [file,location] = uigetfile({'*.*'; '*jpg'}, 'Folder', dirPath, 'MultiSelect', 'on');
    
    % Extract features from frames
    image_points = zeros(3, 2, 3);
    m = {}; % features for DLT
    for i=1:3
        abs_path = fullfile(location,file(i));
        [image_points(i,1,:), image_points(i,2,:), m{i}] = selectFeatures_p2(abs_path{1}, cameraParams.Intrinsics, true);
        disp(['Frame ', num2str(i),': features selected'])
    end
    
    % Save features to be used later on
    save(dirPath+"\precooked.mat", "image_points", "m")
end