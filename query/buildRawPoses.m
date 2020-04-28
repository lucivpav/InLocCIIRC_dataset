addpath('../functions/closest_value');
addpath('../functions/local/projectPointCloud');
addpath('../functions/disperse');
addpath('../functions/InLocCIIRC_utils/mkdirIfNonExistent');

generateMiniSequence = false;

[ params ] = setupParams('holoLens1Params');
miniSequenceDir = fullfile(params.query.dir, 'miniSequence');
mkdirIfNonExistent(miniSequenceDir);

measurementPath = '/Volumes/GoogleDrive/Můj disk/ARTwin/personal/lucivpav/HoloLens sequences/measurement1.txt';

%% prepare Vicon table with millisecond timestamps
measurementTable = readtable(measurementPath);
measurementTable.Properties.VariableNames = {'frameNumber', 'FPS', 'marker', 'invalid', 'x', 'y', 'z', 'alpha', 'beta', 'gamma'};

FPS = 100; % TODO: check with the data that this is true

measurementTable.frameNumber = measurementTable.frameNumber - measurementTable.frameNumber(1);
measurementTable.timestampMs = measurementTable.frameNumber * (1 / FPS) * 1000;
measurementTable = removevars(measurementTable, {'FPS', 'marker', 'frameNumber'});
measurementTable = measurementTable(~measurementTable.invalid, {'timestampMs', 'x', 'y', 'z', 'alpha', 'beta', 'gamma'});

files = dir('/Volumes/GoogleDrive/Můj disk/ARTwin/personal/lucivpav/HoloLens sequences/HoloLensRecording__2020_04_23__09_53_01/pv');
files = files(endsWith({files.name}, '.jpg'));

timestamps = {files.name};
timestamps = extractBetween(timestamps, 1, strlength(timestamps)-4);
timestamps = strcat('uint64(', timestamps, ')');
timestamps = str2num(str2mat(timestamps));
timestamps = (timestamps - timestamps(1)) / 10000;
queryTable = table({files.name}', timestamps);
queryTable.Properties.VariableNames = {'name', 'timestampMs'};

%% try a synchronization constant
syncConstant = 10.7 * 1000; % [ms]

%queryName = '00132321090555753820.jpg';
%queryName = '00132321090868821963.jpg';
%queryName = '00132321091341754646.jpg';
%queryName = '00132321091488297196.jpg';
%queryName = '00132321091075313721.jpg';
%queryName = '00132321091195212118.jpg'; % broken, sadly
queryName = '00132321091305119025.jpg';
queryTimestamp = queryTable(find(strcmp(queryTable.name,queryName)), 'timestampMs');
queryTimestamp = queryTimestamp{1,1};
viconTimestamp = syncConstant + queryTimestamp;

padding = 1500;
tDiffsMs = -padding:100:padding;
if ~generateMiniSequence
    tDiffsMs = [0];
end
for i=1:size(tDiffsMs,2)
tDiffMs = tDiffsMs(1,i);
[~, idx] = closest_value(measurementTable.timestampMs, viconTimestamp+tDiffMs);

%% project and check whether it corresponds to the initial sequence image
closestEvent = measurementTable(idx,:);
rawPosition = [closestEvent{1,'x'}; closestEvent{1,'y'}; closestEvent{1,'z'}];
rawRotation = [closestEvent{1,'alpha'}, closestEvent{1,'beta'}, closestEvent{1,'gamma'}];
[R, t] = rawPoseToPose(rawPosition, rawRotation, params);

pointSize = 8.0;
outputSize = params.camera.sensor.size;
projectedPointCloud = projectPointCloud(params.pointCloud.path, params.camera.fl, R, ...
                                    t, params.camera.sensor.size, outputSize, pointSize, ...
                                    params.projectPointCloudPy.path);
if generateMiniSequence
    projectedPointCloudPath = fullfile(miniSequenceDir, sprintf('%d_%d.jpg',i,tDiffMs));
    imwrite(projectedPointCloud, projectedPointCloudPath);
else
    imshow(projectedPointCloud);
end

end