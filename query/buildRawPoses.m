addpath('../functions/closest_value');
addpath('../functions/local/projectPointCloud');
addpath('../functions/InLocCIIRC_utils/mkdirIfNonExistent');
addpath('../functions/InLocCIIRC_utils/rotationMatrix');

generateMiniSequence = false;

[ params ] = setupParams('holoLens1Params');
miniSequenceDir = fullfile(params.query.dir, 'miniSequence');
mkdirIfNonExistent(miniSequenceDir);

%% prepare Vicon table with millisecond timestamps
measurementTable = readtable(params.measurement.path);
measurementTable.Properties.VariableNames = {'frameNumber', 'FPS', 'marker', 'invalid', 'x', 'y', 'z', 'alpha', 'beta', 'gamma'};

FPS = 100; % TODO: check with the data that this is true

measurementTable.frameNumber = measurementTable.frameNumber - measurementTable.frameNumber(1);
measurementTable.timestampMs = measurementTable.frameNumber * (1 / FPS) * 1000;
measurementTable = removevars(measurementTable, {'FPS', 'marker', 'frameNumber'});
measurementTable = measurementTable(~measurementTable.invalid, {'timestampMs', 'x', 'y', 'z', 'alpha', 'beta', 'gamma'});

queryFiles = dir(params.input.query.dir);
queryFiles = queryFiles(endsWith({queryFiles.name}, '.jpg'));

timestamps = {queryFiles.name};
timestamps = extractBetween(timestamps, 1, strlength(timestamps)-4);
timestamps = strcat('uint64(', timestamps, ')');
timestamps = str2num(str2mat(timestamps));
timestamps = (timestamps - timestamps(1)) / 10000;
queryTable = table({queryFiles.name}', timestamps);
queryTable.Properties.VariableNames = {'name', 'timestampMs'};

%% try a synchronization constant

% HoloLens1
%queryName = '00132321091195212118.jpg'; % broken, sadly
queryName = params.interestingQueries(5);

% HoloLens2
% queryName = params.interestingQueries(3);

queryTimestamp = queryTable(find(strcmp(queryTable.name,queryName)), 'timestampMs');
queryTimestamp = queryTimestamp{1,1};
viconTimestamp = params.HoloLensViconSyncConstant + queryTimestamp;

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

return

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% once quite good parameters are found, run this to find even better parameters nearby (by brute-force search)
padding = 200;
tDiffsMs = -padding:50:padding;
originPadding = 5;
originDiffs = params.camera.originConstant * (-originPadding:1:originPadding);
rotationPadding = 5;
rotationDiffs = -rotationPadding:1:rotationPadding;

%% HoloLens1 specific %%
queryInd = 1:size(params.interestingQueries,2);

%% HoloLens2 specific %%
% %queryInd = 1:size(params.interestingQueries,2);
% %queryInd = [1,2];
% queryInd = [3,4]; % these two should be independent subsequences, due to possible hat shift TODO: verify
% queryInd = [1,2,4];

K = eye(3);
K(1,1) = params.camera.fl;
K(2,2) = params.camera.fl;
K(1,3) = params.camera.sensor.size(2)/2;
K(2,3) = params.camera.sensor.size(1)/2;

bestOrigin = params.camera.origin.wrt.marker;
bestRotation = params.camera.rotation.wrt.marker;
clearvars rawPositions;
clearvars rawRotations;
for i1=1:size(tDiffsMs,2) 
    tDiffMs = tDiffsMs(1,i1);
    syncConstant = params.HoloLensViconSyncConstant + tDiffMs;
    for i2=1:size(queryInd,2)
        queryIdx = queryInd(i2);
        [rawPosition, rawRotation] = buildRawPose(queryIdx, params.interestingQueries, queryTable, measurementTable, syncConstant);
        thisRawPositions{i2} = rawPosition;
        thisRawRotations{i2} = rawRotation;
    end
    rawPositions{i1} = thisRawPositions;
    rawRotations{i1} = thisRawRotations;
end
bestSyncConstantIdx = uint8(round(size(tDiffsMs,2)/2));
thisRawPositions = rawPositions{bestSyncConstantIdx};
thisRawRotations = rawRotations{bestSyncConstantIdx};
lowestError = projectionError(queryInd, bestOrigin, bestRotation, ...
                              params.interestingPointsPC, params.interestingPointsQuery, ...
                              thisRawPositions, thisRawRotations, K, params);

%%
fprintf('Error: %0.2f\n', lowestError);
for i1=1:size(tDiffsMs,2)
    thisRawPositions = rawPositions{i1};
    thisRawRotations = rawRotations{i1};
    for i2=1:size(originDiffs,2)
        for i3=1:size(originDiffs,2)
            for i4=1:size(originDiffs,2)
                originDiff = [originDiffs(1,i2); originDiffs(1,i3); originDiffs(1,i4)];
                origin = params.camera.origin.wrt.marker + originDiff;
                for i5=1:size(rotationDiffs,2)
                    for i6=1:size(rotationDiffs,2)
                        for i7=1:size(rotationDiffs,2)
                            rotationDiff = [rotationDiffs(1,i5), rotationDiffs(1,i6), rotationDiffs(1,i7)];
                            rotation = params.camera.rotation.wrt.marker + rotationDiff;
                            error = projectionError(queryInd, origin, rotation, ...
                                                    params.interestingPointsPC, params.interestingPointsQuery, ...
                                                    thisRawPositions, thisRawRotations, K, params);
                            if error < lowestError
                                lowestError = error;
                                fprintf('Error: %0.2f\n', error);
                                bestSyncConstantIdx = i1;
                                bestOrigin = origin;
                                bestRotation = rotation;
                            end
                        end
                    end
                end
            end
        end
        fprintf('Round %d/%d, %0.0f%% done.\n', i1, size(tDiffsMs,2), i2*100/size(originDiffs,2));
    end
end

%% visualize correspondences and errors
for i=1:size(queryInd,2)
    queryIdx = queryInd(i);
    paramsBak = params;
    params.camera.origin.wrt.marker = bestOrigin;
    params.camera.rotation.wrt.marker = bestRotation;
    thisRawPositions = rawPositions{bestSyncConstantIdx};
    thisRawRotations = rawRotations{bestSyncConstantIdx};
    thisRawPosition = thisRawPositions{i};
    thisRawRotation = thisRawRotations{i};
    projectedInterestingPoints = projectPoints(params.interestingPointsPC{queryIdx}, thisRawPosition, thisRawRotation, K, params);
    [R, t] = rawPoseToPose(thisRawPosition, thisRawRotation, params);
    params = paramsBak;
    thisInterestingPointsQuery = params.interestingPointsQuery{queryIdx};

    figure;
    pointSize = 8.0;
    outputSize = params.camera.sensor.size;
    projectedPointCloud = projectPointCloud(params.pointCloud.path, params.camera.fl, R, ...
                                        t, params.camera.sensor.size, outputSize, pointSize, ...
                                        params.projectPointCloudPy.path);
    image(projectedPointCloud);
    axis image;

    hold on;
    scatter(projectedInterestingPoints(1,:), projectedInterestingPoints(2,:), 40, 'r', 'filled');
    scatter(thisInterestingPointsQuery(1,:), thisInterestingPointsQuery(2,:), 40, 'g', 'filled');
    nCorrespondences = size(thisInterestingPointsQuery,2);
    for i=1:nCorrespondences
        plot([thisInterestingPointsQuery(1,i), projectedInterestingPoints(1,i)], ...
             [thisInterestingPointsQuery(2,i), projectedInterestingPoints(2,i)], ...
             'r-', 'linewidth', 2);
    end
    hold off;
end

return
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% once good enough parameters are found, run this to generate rawPoses.csv
rawPosesFile = fopen(params.rawPoses.path, 'w');
fprintf(rawPosesFile, 'id x y z alpha beta gamma space\n');

for i=1:size(queryTable,1)
    inputQueryPath = fullfile(queryFiles(i).folder, queryFiles(i).name);
    outputQueryPath = fullfile(params.query.dir, sprintf('%d.jpg', i));
    copyfile(inputQueryPath, outputQueryPath);
    
    queryTimestamp = queryTable(i, 'timestampMs');
    queryTimestamp = queryTimestamp{1,1};
    viconTimestamp = params.HoloLensViconSyncConstant + queryTimestamp;
    [~, idx] = closest_value(measurementTable.timestampMs, viconTimestamp);
    closestEvent = measurementTable(idx,:);
    space = 'B-315';
    fprintf(rawPosesFile, '%d %f %f %f %f %f %f %s\n', ...
        i, closestEvent{1,'x'}, closestEvent{1,'y'}, closestEvent{1,'z'}, ...
        closestEvent{1,'alpha'}, closestEvent{1,'beta'}, closestEvent{1,'gamma'}, ...
        space);
end

fclose(rawPosesFile);