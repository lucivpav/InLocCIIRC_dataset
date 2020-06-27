addpath('../functions/closest_value');
addpath('../functions/local/projectPointCloud');
addpath('../functions/InLocCIIRC_utils/mkdirIfNonExistent');
addpath('../functions/InLocCIIRC_utils/rotationMatrix');

justEvaluateOnMatches = false;
generateMiniSequence = false;

[ params ] = setupParams('holoLens1Params'); % WARNING: if you change params in the file and run this again,
                                             % the new params may not actually load on first attempt, making the evaluation result
					                         % unexpected. IDE bug?
miniSequenceDir = fullfile(params.query.dir, 'miniSequence');
mkdirIfNonExistent(miniSequenceDir);

[measurementTable, queryTable, queryFiles] = initiMeasurementAndQueryTables(params);

if justEvaluateOnMatches
    close all
    queryInd = 1:size(params.interestingQueries,2);
    %queryInd = [3];
    evaluateMatches(queryInd, params, queryTable, measurementTable, false);
    return;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% once quite good parameters are found, run this to find even better parameters nearby (by brute-force search)
close all
tDiffsMs = -200:100:200;
originPadding = 8;
originDiffs = -originPadding:1:originPadding;
rotationPadding = 5;
rotationDiffs = -rotationPadding:0.5:rotationPadding;
errors = inf(size(tDiffsMs,2)*size(originDiffs,2), size(originDiffs,2), size(originDiffs,2), ...
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
    for i2=1:size(queryInd,2)
        queryIdx = queryInd(i2);
        paramsCopy = params;
        paramsCopy.HoloLensViconSyncConstant = paramsCopy.HoloLensViconSyncConstant +tDiffMs;
        [rawPosition, rawRotation] = getRawPose(queryIdx, params.interestingQueries, queryTable, measurementTable, false, paramsCopy);
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

nTDiffs = size(tDiffsMs,2);
[topLoop1Ind, topLoop2Ind] = ndgrid(1:nTDiffs, 1:nOriginDiffs);
topLoopsInd = [topLoop1Ind(:), topLoop2Ind(:)];

env = environment();
if strcmp(env, 'laptop')
    nWorkers = 4;
elseif strcmp(env, 'cmp')
    nWorkers = 45;
end
c = parcluster;
c.NumWorkers = nWorkers;
saveProfile(c);

p = parpool('local', nWorkers);
opts = parforOptions(p);
nTopLoopsInd = size(topLoopsInd,1);
parfor iTop=1:nTopLoopsInd
    fprintf('Processing round %d/%d.\n', iTop, nTopLoopsInd);
    i1 = topLoopsInd(iTop,1);
    i2 = topLoopsInd(iTop,2);
    thisRawPositions = rawPositions{i1};
    thisRawRotations = rawRotations{i1};
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
                        errors(iTop,i3,i4,i5,i6,i7) = sum(error);
                    end
                end
            end
        end
    end
end

errors2 = inf(size(tDiffsMs,2), ...
		size(originDiffs,2), size(originDiffs,2), size(originDiffs,2), ...
                size(rotationDiffs,2), size(rotationDiffs,2), size(rotationDiffs,2));
for i1=1:nTDiffs
    for i2=1:nOriginDiffs
	[q,iTop] = ismember([i1,i2], topLoopsInd, 'rows');
        for i3=1:nOriginDiffs
        for i4=1:nOriginDiffs
        for i5=1:nRotationDiffs
        for i6=1:nRotationDiffs
        for i7=1:nRotationDiffs
	errors2(i1,i2,i3,i4,i5,i6,i7) = errors(iTop,i3,i4,i5,i6,i7); % TODO: get rid of the inner loops
        end
        end
        end
        end
        end
    end
end
errors = errors2;

%%
close all
for i1=1:size(tDiffsMs,2)
    thisErrors = errors(i1,:);
    thisErrors = reshape(thisErrors, max(size(thisErrors)), 1);
    [lowestError,idx] = min(thisErrors);
    dimensionSizes = size(errors);
    [i2,i3,i4,i5,i6,i7] = ind2sub(dimensionSizes(2:end),idx);
    originDiff = [originDiffs(1,i2); originDiffs(1,i3); originDiffs(1,i4)];
    bestOrigin = params.camera.originConstant * (params.camera.origin.relative.wrt.marker + originDiff);
    rotationDiff = [rotationDiffs(1,i5), rotationDiffs(1,i6), rotationDiffs(1,i7)];
    bestRotation = params.camera.rotation.wrt.marker + rotationDiff;
    bestSyncConstantIdx = i1;

    optimalParams{i1} = params;
    optimalParams{i1}.camera.origin.wrt.marker = bestOrigin;
    optimalParams{i1}.camera.origin.relative.wrt.marker = bestOrigin / params.camera.originConstant;
    optimalParams{i1}.camera.rotation.wrt.marker = bestRotation;
    optimalParams{i1}.HoloLensViconSyncConstant = optimalParams{i1}.HoloLensViconSyncConstant + tDiffsMs(bestSyncConstantIdx);
end

[lowestError,idx] = min(errors(:));
[i1,i2,i3,i4,i5,i6,i7] = ind2sub(size(errors),idx);
evaluateMatches(queryInd, optimalParams{i1}, queryTable, measurementTable, false);

save(fullfile(params.query.dir, 'bruteForce.mat'), 'optimalParams', 'errors', '-v7.3');

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
    viconTimestamp = double(params.HoloLensViconSyncConstant + queryTimestamp);
    [~, idx] = closest_value(measurementTable.timestampMs, viconTimestamp);
    closestEvent = measurementTable(idx,:);
    space = 'B-315';
    fprintf(rawPosesFile, '%d %f %f %f %f %f %f %s\n', ...
        i, closestEvent{1,'x'}, closestEvent{1,'y'}, closestEvent{1,'z'}, ...
        closestEvent{1,'alpha'}, closestEvent{1,'beta'}, closestEvent{1,'gamma'}, ...
        space);
end

fclose(rawPosesFile);
