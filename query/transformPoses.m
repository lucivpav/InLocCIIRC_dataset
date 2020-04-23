addpath('../functions/local/projectPointCloud');
addpath('../functions/InLocCIIRC_utils/rotationMatrix');
addpath('../functions/InLocCIIRC_utils/mkdirIfNonExistent');
addpath('../functions/InLocCIIRC_utils/P_to_str')
addpath('../functions/local/R_to_numpy_array')
[ params ] = setupParams('s10e');

mkdirIfNonExistent(params.projectedPointCloud.dir);
mkdirIfNonExistent(params.poses.dir);

rawPosesTable = readtable(params.rawPoses.path);
descriptionsFile = fopen(params.queryDescriptions.path, 'w');

fprintf(descriptionsFile, 'id space inMap\n');

cutoutDescriptions = buildCutoutDescriptions(params);

mkdirIfNonExistent(params.closest.cutout.dir);

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

    %% this stuff must be extracted into a function

    viconOrigin = [-0.13; 0.04; 2.80]; % w.r.t. model
    viconRotation = deg2rad([90.0 180.0 0.0]); % w.r.t. model

    cameraRotation = params.camera.rotation.wrt.marker;
    
    % TODO: why does the rotation orientation have to be flipped?
    % NOTE: this is necessary for correctness
    % NOTE: this probably forces the ugly hack later on
    markerRotation = rawRotation .* [1.0 -1.0 1.0];

    cameraR = rotationMatrix(cameraRotation, 'XYZ');
    markerR = rotationMatrix(markerRotation, 'ZYX');
    viconR = rotationMatrix(viconRotation, 'XYZ');

    markerOrigin = viconR * rawPosition + viconOrigin; % w.r.t. model

    % note: coordinate vectors are columns
    markerCoordinateSystem = markerR * eye(3); % w.r.t. vicon
    markerCoordinateSystem = viconR * markerCoordinateSystem; % w.r.t. model
    
    cameraOrigin = params.camera.origin.wrt.marker;
    cameraOrigin = markerCoordinateSystem * cameraOrigin + markerOrigin; % w.r.t. marker, from model origin

    markerCoordinateSystem = markerR * eye(3); % w.r.t. vicon
    cameraCoordinateSystem = cameraR * markerCoordinateSystem; % w.r.t. vicon 
    cameraCoordinateSystem = viconR * cameraCoordinateSystem; % w.r.t. model, camera points to y
    
    f = params.camera.fl; % in pixels
    sensorSize = params.camera.sensor.size; % height, width
    
    outputSize = sensorSize;
    
    % bring z to where y is, as required by projectPC
    rFix = rotationMatrix([-pi/2, 0.0, 0.0], 'ZYX');
    cameraCoordinateSystem = cameraCoordinateSystem * rFix;
    %RprojectPC = eye(3);

    t = cameraOrigin;
    P = eye(4);
    P(1:3,1:3) = cameraCoordinateSystem;
    P(1:3,4) = cameraCoordinateSystem * -t;

    poseFile = fopen(fullfile(params.poses.dir, sprintf('%d.txt', id)), 'w');
    P_str = P_to_str(P);
    fprintf(poseFile, '%s', P_str);
    fclose(poseFile);

    pointSize = 8.0;
    
    projectedPointCloud = projectPointCloud(params.pointCloud.path, f, cameraCoordinateSystem, ...
                                        t, sensorSize, outputSize, pointSize, ...
                                        params.projectPointCloudPy.path);
    
    %% end of extraction

    imshow(projectedPointCloud);
    queryFilename = sprintf('%d.jpg', id);
    projectedPointCloudFile = fullfile(params.projectedPointCloud.dir, queryFilename);
    imwrite(projectedPointCloud, projectedPointCloudFile);
    
    [inMap, closestCutout] = isQueryInMap(P, params.spaceName, cutoutDescriptions, params);
    if params.renderClosestCutouts
        queryImg = imread(fullfile(params.query.dir, queryFilename));
        cutoutImg = imresize(closestCutout.img, [size(queryImg,1), size(queryImg,2)]);
        imshowpair(queryImg, cutoutImg, 'montage');
        caption = 'Left: query. Right: closest cutout';
        if inMap
            inMapDesc = 'InMap';
        else
            inMapDesc = 'OffMap';
        end
        tDiff = closestCutout.tDiff;
        rotDist = closestCutout.rotDist;
        title(sprintf('Query %d: %s. tDiff: %0.2f, rotDist: %0.2f. %s - %s, %s.', ...
            id, inMapDesc, tDiff, rotDist, caption, closestCutout.name, closestCutout.space), ...
            'Interpreter', 'none');
        closestCutoutPath = fullfile(params.closest.cutout.dir, queryFilename);
        saveas(gcf, closestCutoutPath);
    end

    fprintf(descriptionsFile, '%d %s %d\n', id, space, inMap);
end

fclose(descriptionsFile);