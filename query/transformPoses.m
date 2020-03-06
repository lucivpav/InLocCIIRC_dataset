addpath('../projectPointCloud');
addpath('../rotationMatrix');
[ params ] = setupParams;

if exist(params.projectedPointCloud.dir, 'dir') ~= 7
    mkdir(params.projectedPointCloud.dir);
end

rawPosesTable = readtable(params.rawPoses.path);
posesFile = fopen(params.poses.path, 'w');

fprintf(posesFile, 'id x y z dirx diry dirz\n');

for i=1:size(rawPosesTable,1)

    id = rawPosesTable{i, 'id'};
    x = rawPosesTable{i, 'x'};
    y = rawPosesTable{i, 'y'};
    z = rawPosesTable{i, 'z'};
    alpha = rawPosesTable{i, 'alpha'};
    beta = rawPosesTable{i, 'beta'};
    gamma = rawPosesTable{i, 'gamma'};
    
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
    R2 = rotationMatrix([-pi/2, 0.0, 0.0], 'ZYX'); % aka rFix, black magic
    t = cameraOrigin;
    pointSize = 8.0;
    projectedPointCloud = projectPointCloud(params.pointCloud.path, f, R * R2, ...
                                        t, sensorSize, outputSize, pointSize, ...
                                        params.projectPointCloudPy.path);
    projectedPointCloudFile = fullfile(params.projectedPointCloud.dir, sprintf('%d.jpg', id));
    imwrite(projectedPointCloud, projectedPointCloudFile);
    
    dir = cameraCoordinateSystem(:,2);
    fprintf(posesFile, '%d %g %g %g %g %g %g\n', id, t(1), t(2), t(3), ...
            dir(1), dir(2), dir(3));
end

fclose(posesFile);