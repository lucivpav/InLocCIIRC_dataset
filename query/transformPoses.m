addpath('../projectPointCloud');
addpath('../functions/InLocCIIRC_utils/rotationMatrix');
[ params ] = setupParams;

if exist(params.projectedPointCloud.dir, 'dir') ~= 7
    mkdir(params.projectedPointCloud.dir);
end

rawPosesTable = readtable(params.rawPoses.path);
posesFile = fopen(params.poses.path, 'w');

fprintf(posesFile, 'id x y z dirx diry dirz space inMap\n');

cutoutDescriptions = buildCutoutDescriptions(params);
if exist(params.closest.cutout.dir, 'dir') ~= 7
    mkdir(params.closest.cutout.dir);
end

for i=1:size(rawPosesTable,1)

    id = rawPosesTable{i, 'id'};
    x = rawPosesTable{i, 'x'};
    y = rawPosesTable{i, 'y'};
    z = rawPosesTable{i, 'z'};
    alpha = rawPosesTable{i, 'alpha'};
    beta = rawPosesTable{i, 'beta'};
    gamma = rawPosesTable{i, 'gamma'};
    space = rawPosesTable{i, 'space'}{1,1};
    
    rawPosition = [x; y; z];
    rawRotation = [alpha beta gamma]; % in radians

    viconOrigin = [-0.13; 0.04; 2.80]; % w.r.t. model
    viconRotation = deg2rad([90.0 180.0 0.0]); % w.r.t. model

    cameraRotation = deg2rad([0.0 0.0 1.5]); % w.r.t. marker

    % TODO: why does the rotation orientation have to be flipped?
    markerRotation = rawRotation .* [1.0 -1.0 1.0];

    cameraR = rotationMatrix(cameraRotation, 'XYZ');
    markerR = rotationMatrix(markerRotation, 'ZYX');
    viconR = rotationMatrix(viconRotation, 'XYZ');

    markerOrigin = viconR * rawPosition + viconOrigin; % w.r.t. model

    % note: coordinate vectors are columns
    markerCoordinateSystem = markerR * eye(3); % w.r.t. vicon
    markerCoordinateSystem = viconR * markerCoordinateSystem; % w.r.t. model

    cameraOrigin = 0.01 * [-3; 1; -4]; % w.r.t. marker, from marker origin
    cameraOrigin = markerCoordinateSystem * cameraOrigin + markerOrigin; % w.r.t. marker, from model origin

    markerCoordinateSystem = markerR * eye(3); % w.r.t. vicon
    cameraCoordinateSystem = cameraR * markerCoordinateSystem; % w.r.t. vicon 
    cameraCoordinateSystem = viconR * cameraCoordinateSystem; % w.r.t. model

    pc = pcread(params.pointCloud.path);
    f = 3172.435; % in pixels
    sensorSize = [3024 4032];
    outputSize = sensorSize;

    R = cameraCoordinateSystem;

    %%
    % y is the camera dir, but rendered is z dir
    % i.e. this will bring z to where y currently is
    rFix = rotationMatrix([-pi/2, 0.0, 0.0], 'ZYX');
    t = cameraOrigin;
    pointSize = 8.0;
    projectedPointCloud = projectPointCloud(params.pointCloud.path, f, R * rFix, ...
                                        t, sensorSize, outputSize, pointSize, ...
                                        params.projectPointCloudPy.path);
    imshow(projectedPointCloud);
    projectedPointCloudFile = fullfile(params.projectedPointCloud.dir, sprintf('%d.jpg', id));
    imwrite(projectedPointCloud, projectedPointCloudFile);
    
    orient = R(:,2); % somehow, this contains the 180.0 rFix, which is wrong
    % very ugly magic hack
    if orient(3) < 0
        orient = orient .* [1.0; -1.0; 1.0];
    end

    [inMap, closestCutout] = isQueryInMap(t, orient, params.spaceName, cutoutDescriptions, params);
    if params.renderClosestCutouts
        queryImg = imread(fullfile(params.query.dir, sprintf('%d.jpg', id)));
        cutoutImg = imresize(closestCutout.img, [size(queryImg,1), size(queryImg,2)]);
        imshowpair(queryImg, cutoutImg, 'montage');
        caption = 'Left: query. Right: closest cutout';
        if inMap
            inMapDesc = 'InMap';
        else
            inMapDesc = 'OffMap';
        end
        tDiff = closestCutout.tDiff;
        dirDiff = closestCutout.dirDiff;
        title(sprintf('Query %d: %s. tDiff: %0.2f, dirDiff: %0.2f. %s - %s, %s.', ...
            id, inMapDesc, tDiff, dirDiff, caption, closestCutout.name, closestCutout.space), ...
            'Interpreter', 'none');
        closestCutoutPath = fullfile(params.closest.cutout.dir, sprintf('%d.jpg', id));
        saveas(gcf, closestCutoutPath);
    end
    
    fprintf(posesFile, '%d %g %g %g %g %g %g %s %d\n', id, t(1), t(2), t(3), ...
            orient(1), orient(2), orient(3), space, inMap);
end

fclose(posesFile);