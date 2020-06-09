addpath('../functions/closest_value');
addpath('../functions/local/projectPointCloud');
addpath('../functions/local/reconstructPose');
addpath('../functions/InLocCIIRC_utils/mkdirIfNonExistent');
addpath('../functions/InLocCIIRC_utils/rotationMatrix');

justEvaluateOnMatches = false;

[ params ] = setupParams('holoLens1Params');

%% prepare Vicon table with millisecond timestamps
measurementTable = readtable(params.measurement.path);
measurementTable.Properties.VariableNames = {'frameNumber', 'FPS', 'marker', 'invalid', 'x', 'y', 'z', 'alpha', 'beta', 'gamma'};

FPS = 100; % viz each row. TODO: assert

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

queryInd = 1:size(params.interestingQueries,2);
if justEvaluateOnMatches
    close all
    evaluateMatches(queryInd, params, queryTable, measurementTable); % TODO
    return
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% once quite good parameters are found, run this to find even better parameters nearby (by brute-force search)
close all

% TODO: try different sync constants
syncConstant = params.HoloLensViconSyncConstant; % TODO: try more in neighborhood
nInterestingQueries = size(queryInd,2);
for i=1:nInterestingQueries
    queryIdx = queryInd(i);
    [R,C] = reconstructPose(params.interestingPointsQuery{queryIdx}, params.interestingPointsPC{queryIdx}, ...
                            params.K, params.reconstructPosePy.path);
    referenceRotations{i} = R; % wrt model
    referencePositions{i} = C; % wrt model
end
%evaluateMatchesUsingPs(queryInd, params, referenceRotations, referencePositions);

%%
for i=1:nInterestingQueries
    queryIdx = queryInd(i);
    [rawPosition, rawRotation] = buildRawPose(queryIdx, params.interestingQueries, queryTable, measurementTable, syncConstant);
    rawPositions{i} = rawPosition; % aka vicon beacon position wrt vicon
end

%%
A = eye(4);
[d,Z,transform] = procrustes(cell2mat(referencePositions)', ...
                                cell2mat(rawPositions)', ...
                                'scaling', true, 'reflection', false);
R = transform.T;
A(1:3,1:3) = R; % NOTE: first, R must be correct, then t can be correct
A(1:3,4) = -R*transform.c(1,:)';

for i=1:nInterestingQueries
    queryIdx = queryInd(i);
    [rawPosition, rawRotation] = buildRawPose(queryIdx, params.interestingQueries, queryTable, measurementTable, syncConstant);
    markerRotation = rawRotation .* [1.0 -1.0 1.0];
    markerR = rotationMatrix(markerRotation, 'ZYX');
    markerBases = markerR * eye(3); % w.r.t. vicon
    markerOrigin = rawPosition; % w.r.t. vicon
    P = [params.K * markerBases, -params.K * markerBases * transform.b * markerOrigin];
    P = P * A; % why is this not A * P? because we are transforming the vicon coorcinate system into model coordinate system
    estimatedRotations{i} = inv(params.K) * P(1:3,1:3); % incorrect
    estimatedPositions{i} = -inv(P(1:3,1:3)) * P(1:3,4); % correct (matches Z)
end
evaluateMatchesUsingPs(queryInd, params, estimatedRotations, estimatedPositions);


return;

tDiffsMs = -200:100:200;
originPadding = 10;
originDiffs = -originPadding:2:originPadding;
rotationPadding = 10;
rotationDiffs = -rotationPadding:0.5:rotationPadding;
errors = inf(size(tDiffsMs,2), ...
                size(originDiffs,2), size(originDiffs,2), size(originDiffs,2), ...
                size(rotationDiffs,2), size(rotationDiffs,2), size(rotationDiffs,2));

%% HoloLens1 specific %%
queryInd = 1:size(params.interestingQueries,2);

%% HoloLens2 specific %%
% %queryInd = 1:size(params.interestingQueries,2);
% %queryInd = [1,2];
% queryInd = [3,4]; % these two should be independent subsequences, due to possible hat shift TODO: verify
% queryInd = [1,2,4];

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
bestSyncConstantIdx = find(tDiffsMs == 0);
thisRawPositions = rawPositions{bestSyncConstantIdx};
thisRawRotations = rawRotations{bestSyncConstantIdx};
lowestError = projectionError(queryInd, bestOrigin, bestRotation, ...
                              params.interestingPointsPC, params.interestingPointsQuery, ...
                              thisRawPositions, thisRawRotations, params);

%%
fprintf('Error: %0.2f\n', lowestError);
nOriginDiffs = size(originDiffs,2);
nRotationDiffs = size(rotationDiffs,2);
p = parpool;
opts = parforOptions(p);
parfor i1=1:size(tDiffsMs,2)
    fprintf('Processing round %d/%d.\n', i1, size(tDiffsMs,2));
    thisRawPositions = rawPositions{i1};
    thisRawRotations = rawRotations{i1};
    for i2=1:nOriginDiffs
        for i3=1:nOriginDiffs
            for i4=1:nOriginDiffs
                originDiff = [originDiffs(1,i2); originDiffs(1,i3); originDiffs(1,i4)];
                origin = params.camera.originConstant * (params.camera.origin.relative.wrt.marker + originDiff);
                for i5=1:nRotationDiffs
                    for i6=1:nRotationDiffs
                        for i7=1:nRotationDiffs
                            rotationDiff = [rotationDiffs(1,i5), rotationDiffs(1,i6), rotationDiffs(1,i7)];
                            rotation = params.camera.rotation.wrt.marker + rotationDiff;
                            error = projectionError(queryInd, origin, rotation, ...
                                                    params.interestingPointsPC, params.interestingPointsQuery, ...
                                                    thisRawPositions, thisRawRotations, params);
                            errors(i1,i2,i3,i4,i5,i6,i7) = sum(error);
                        end
                    end
                end
            end
        end
    end
end

%%
close all
[lowestError,idx] = min(errors(:));
[i1,i2,i3,i4,i5,i6,i7] = ind2sub(size(errors),idx);
originDiff = [originDiffs(1,i2); originDiffs(1,i3); originDiffs(1,i4)];
bestOrigin = params.camera.originConstant * (params.camera.origin.relative.wrt.marker + originDiff);
rotationDiff = [rotationDiffs(1,i5), rotationDiffs(1,i6), rotationDiffs(1,i7)];
bestRotation = params.camera.rotation.wrt.marker + rotationDiff;
bestSyncConstantIdx = i1;

optimalParams = params;
optimalParams.camera.origin.wrt.marker = bestOrigin;
optimalParams.camera.origin.relative.wrt.marker = bestOrigin / params.camera.originConstant;
optimalParams.camera.rotation.wrt.marker = bestRotation;
optimalParams.HoloLensViconSyncConstant = optimalParams.HoloLensViconSyncConstant + tDiffsMs(bestSyncConstantIdx);
evaluateMatches(queryInd, optimalParams, queryTable, measurementTable);

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