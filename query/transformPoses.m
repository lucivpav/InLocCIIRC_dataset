addpath('../functions/local/projectPointCloud');
addpath('../functions/InLocCIIRC_utils/rotationMatrix');
addpath('../functions/InLocCIIRC_utils/mkdirIfNonExistent');
addpath('../functions/InLocCIIRC_utils/P_to_str')
addpath('../functions/local/R_to_numpy_array')
[ params ] = setupParams('holoLens1Params');

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

    [R, t] = rawPoseToPose(rawPosition, rawRotation, params);

    P = eye(4);
    P(1:3,1:3) = R;
    P(1:3,4) = R * -t;

    poseFile = fopen(fullfile(params.poses.dir, sprintf('%d.txt', id)), 'w');
    P_str = P_to_str(P);
    fprintf(poseFile, '%s', P_str);
    fclose(poseFile);

    pointSize = 8.0;
    f = params.camera.fl; % in pixels
    sensorSize = params.camera.sensor.size; % height, width
    outputSize = sensorSize;
    projectedPointCloud = projectPointCloud(params.pointCloud.path, f, R, ...
                                        t, sensorSize, outputSize, pointSize, ...
                                        params.projectPointCloudPy.path);

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