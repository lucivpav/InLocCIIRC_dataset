addpath('../functions/InLocCIIRC_utils/projectMesh');
addpath('../functions/InLocCIIRC_utils/rotationMatrix');
addpath('../functions/InLocCIIRC_utils/mkdirIfNonExistent');
addpath('../functions/local/P_to_str')
addpath('../functions/local/R_to_numpy_array')
[ params ] = setupParams;

mkdirIfNonExistent(params.projectedMesh.dir);
mkdirIfNonExistent(params.poses.dir);

rawPosesTable = readtable(params.rawPoses.path);
descriptionsFile = fopen(params.queryDescriptions.path, 'w');

fprintf(descriptionsFile, 'id space inMap\n');

cutoutDescriptions = buildCutoutDescriptions(params);

mkdirIfNonExistent(params.closest.cutout.dir);

for i=1:size(rawPosesTable,1)
 %   i=40;

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

    cameraOrigin = 0.01 * [-3; 1; -4]; % w.r.t. marker, from marker origin
    cameraOrigin = markerCoordinateSystem * cameraOrigin + markerOrigin; % w.r.t. marker, from model origin

    markerCoordinateSystem = markerR * eye(3); % w.r.t. vicon
    cameraCoordinateSystem = cameraR * markerCoordinateSystem; % w.r.t. vicon 
    cameraCoordinateSystem = viconR * cameraCoordinateSystem; % w.r.t. model, camera points to y
    
    % somehow, this contains the 180.0 rFix, which is wrong
    % very ugly magic hack
    cameraCoordinateSystem2 = cameraCoordinateSystem;
    if cameraCoordinateSystem(2,3) < 0 % TODO: this condition is not 100% correct
        rFix = rotationMatrix([pi/2, 0.0, 0.0], 'ZYX');
        cameraCoordinateSystem2 = cameraCoordinateSystem2 * rFix;
       % cameraCoordinateSystem2(2,:) = -cameraCoordinateSystem(2,:);
%         yBasis = cameraCoordinateSystem(:,2);
%         newYBasis = yBasis .* [1.0; -1.0; 1.0];
%         angle = atan2d(norm(cross(yBasis,newYBasis)),dot(yBasis,newYBasis)); % [deg] this function is correct
%         rFix = rotationMatrix([-deg2rad(angle), 0.0, 0.0], 'ZYX');
%         cameraCoordinateSystem = cameraCoordinateSystem * rFix; % <- here is surely a problem
    end
    
    rFix = rotationMatrix([-pi/2, 0.0, 0.0], 'ZYX');
    cameraCoordinateSystem = cameraCoordinateSystem * rFix; % w.r.t. model, camera points to -z (correctly)
    
    % this is not correct, it does not match score.csv!
    %rFix = rotationMatrix([pi/2, 0.0, 0.0], 'ZYX');
    %cameraCoordinateSystem2 = cameraCoordinateSystem2 * rFix;
    
    pc = pcread(params.pointCloud.path);
    
    f = 3172.435; % in pixels
    sensorSize = [3024, 4032]; % height, width
    
    outputSize = sensorSize;

    R = cameraCoordinateSystem2;
    t = cameraOrigin;
    P = eye(4);
    P(1:3,1:3) = R;
    P(1:3,4) = R * -t;

    poseFile = fopen(fullfile(params.poses.dir, sprintf('%d.txt', id)), 'w');
    P_str = P_to_str(P);
    fprintf(poseFile, '%s', P_str);
    fclose(poseFile);

    rFix = rotationMatrix([-pi, 0.0, 0.0], 'ZYX'); % ???
    pointSize = 8.0;
    R = cameraCoordinateSystem; % TODO: compare R with master's R?
    projectedPointCloud = projectPointCloud(params.pointCloud.path, f, R, ...
                                        t, sensorSize, outputSize, pointSize, ...
                                        params.projectPointCloudPy.path);
%     headless = false; % TODO: adjust to pick headless mode on a server
%     scaling = 4;
    % y is the camera dir, but rendered is z dir
    % i.e. this will bring z to where y currently is
    rFix = rotationMatrix([-pi, 0.0, 0.0], 'ZYX');
    projectedMesh = projectedPointCloud; % TODO
%     sensorSize = [sensorSize(2), sensorSize(1)];
%     
%     % change of sensorSize -> adjust focal length to maintain FoV
%     FoV = deg2rad(64.87); % horizontal FoV
%     sensorWidth = sensorSize(2)/scaling;
%     f = sensorWidth / (2*tan(FoV/2)); % NOTE: this is currently incorrect
%     
%     [projectedMesh, ~, ~] = projectMesh(params.mesh.path, f, R, t, sensorSize/scaling, ...
%                                         false, -1, params.projectMeshPy.path, headless);
%     projectedMesh = imresize(projectedMesh, scaling);
    imshow(projectedMesh);
    queryFilename = sprintf('%d.jpg', id);
    projectedMeshFile = fullfile(params.projectedMesh.dir, queryFilename);
    imwrite(projectedMesh, projectedMeshFile);
    
    % TODO
%     orient = R(:,2); % somehow, this contains the 180.0 rFix, which is wrong
%     % very ugly magic hack
%     if orient(3) < 0
%         orient = orient .* [1.0; -1.0; 1.0];
%     end
    
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