addpath('../functions/closest_value');
addpath('../functions/local/projectPointCloud');

measurementPath = '/Volumes/GoogleDrive/Můj disk/ARTwin/personal/lucivpav/HoloLens sequences/measurement1.txt';

%% prepare Vicon table with millisecond timestamps
measurementTable = readtable(measurementPath);
measurementTable.Properties.VariableNames = {'frameNumber', 'FPS', 'marker', 'invalid', 'x', 'y', 'z', 'alpha', 'beta', 'gamma'};

FPS = 100; % TODO: check with the data that this is true

measurementTable.frameNumber = measurementTable.frameNumber - measurementTable.frameNumber(1);
measurementTable.timestampMs = measurementTable.frameNumber * (1 / FPS) * 1000;
measurementTable = removevars(measurementTable, {'FPS', 'marker', 'frameNumber'});
measurementTable = measurementTable(~measurementTable.invalid, {'timestampMs', 'x', 'y', 'z', 'alpha', 'beta', 'gamma'});

%% try a synchronization constant
syncConstant = 15 * 1000; % [ms]
[~, idx] = closest_value(measurementTable.timestampMs, syncConstant);

%% project and check whether it corresponds to the initial sequence image
closestEvent = measurementTable(idx,:);
rawPosition = [closestEvent{1,'x'}; closestEvent{1,'y'}; closestEvent{1,'z'}];
rawRotation = [closestEvent{1,'alpha'}, closestEvent{1,'beta'}, closestEvent{1,'gamma'}];
[ params ] = setupParams('holoLens1Params');
[R, t] = rawPoseToPose(rawPosition, rawRotation, params);

pointSize = 8.0;
outputSize = params.camera.sensor.size;
projectedPointCloud = projectPointCloud(params.pointCloud.path, params.camera.fl, R, ...
                                    t, params.camera.sensor.size, outputSize, pointSize, ...
                                    params.projectPointCloudPy.path);
imshow(projectedPointCloud);