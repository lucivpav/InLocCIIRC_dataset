addpath('../functions/local/reconstructPose');
addpath('../functions/local/R_to_numpy_array');

close all

[ params ] = setupParams('holoLens1Params');

%% find ground truth camera poses
% TODO: try different sync constants?
syncConstant = params.HoloLensViconSyncConstant;
queryInd = 1:size(params.interestingQueries,2);
nInterestingQueries = size(queryInd,2);
for i=1:nInterestingQueries
    queryIdx = queryInd(i);
    [R,C] = reconstructPose(params.interestingPointsQuery{queryIdx}, params.interestingPointsPC{queryIdx}, ...
                            params.K, params.reconstructPosePy.path);
    gtRotations{i} = R; % wrt model % TODO: rename, this is not actually gt, but  something like cameraRotations{i}
    gtPositions{i} = C; % wrt model % TODO: same here
end

[measurementTable, queryTable, queryFiles] = initiMeasurementAndQueryTables(params);

%% check p3p actually works
%evaluateMatchesUsingPs(queryInd, params, gtRotations, gtPositions);

%% find marker poses
for i=1:nInterestingQueries
    queryIdx = queryInd(i);
    queryName = params.interestingQueries(queryIdx);

    queryTimestamp = queryTable(find(strcmp(queryTable.name,queryName)), 'timestampMs');
    queryTimestamp = queryTimestamp{1,1};
    viconTimestamp = double(params.HoloLensViconSyncConstant + queryTimestamp);

    tDiffMs = 0;
    [~, idx] = closest_value(measurementTable.timestampMs, viconTimestamp+tDiffMs);

    %% project and check whether it corresponds to the initial sequence image
    closestEvent = measurementTable(idx,:);
    rawPosition = [closestEvent{1,'x'}; closestEvent{1,'y'}; closestEvent{1,'z'}];
    rawRotation = [closestEvent{1,'alpha'}, closestEvent{1,'beta'}, closestEvent{1,'gamma'}];
    paramsCopy = params;
    paramsCopy.camera.origin.relative.wrt.marker = [0; 0; 0];
    paramsCopy.camera.origin.wrt.marker = [0; 0; 0];
    paramsCopy.camera.rotation.wrt.marker = [0.0 0.0 0.0];
    [R, t] = rawPoseToPose(rawPosition, rawRotation, paramsCopy);
    markerRotations{i} = R; % wrt model
    markerPositions{i} = t; % wrt model
end

%% find the individual (optimal) params
optimalParams = params;
for i=1:nInterestingQueries
    queryIdx = queryInd(i);

    % bring y to where z is, (undo the format required by projection)
    rFix = rotationMatrix([pi/2, 0.0, 0.0], 'ZYX');
    markerRotation = markerRotations{i} * rFix;
    gtRotation = gtRotations{i} * rFix;

    optimalTranslationsNonRelative{i} = inv(markerRotation) * (gtPositions{i} - markerPositions{i});
    optimalTranslations{i} = optimalTranslationsNonRelative{i} / params.camera.originConstant;

    optimalR = inv(markerRotation) * gtRotation;
    optimalRotations{i} = rad2deg(rotm2eul(optimalR, 'XYZ')); % TODO: WTF, shouldn't this be ZYX?!

    testParams = optimalParams;
    testParams.camera.origin.relative.wrt.marker = optimalTranslations{i};
    testParams.camera.origin.wrt.marker = optimalTranslationsNonRelative{i};
    testParams.camera.rotation.wrt.marker = optimalRotations{i};

    evaluateMatches([queryIdx], testParams, queryTable, measurementTable);
end
optimalParams.optimal.camera.origin.relative.wrt.marker = containers.Map(queryInd, optimalTranslations);
optimalParams.optimal.camera.rotation.wrt.marker = containers.Map(queryInd, optimalRotations);

%% suggest a generic transformation
optimalGenericOrigin = mean(cell2mat(optimalTranslations)');
optimalGenericRotation = mean(cell2mat(optimalRotations'));

optimalParams.camera.origin.relative.wrt.marker = optimalGenericOrigin';
optimalParams.camera.origin.wrt.marker = optimalGenericOrigin' / params.camera.originConstant;
optimalParams.camera.rotation.wrt.marker = optimalGenericRotation;