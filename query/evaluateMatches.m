function evaluateMatches(queryInd, params, queryTable, measurementTable, rawPosesTable)
    % either queryTable, measurementTable are set, or
    % rawPosesTable is set
    % those values that are not set, should be set to false

if islogical(rawPosesTable) && rawPosesTable == false
    useRawPosesTable = false;
else
    useRawPosesTable = true;
end

nQueries = size(queryInd,2);
for i=1:nQueries
    queryIdx = queryInd(i);
    if useRawPosesTable
        [rawPosition, rawRotation] = getRawPose(queryIdx, params.interestingQueries, rawPosesTable);
    else
        [rawPosition, rawRotation] = buildRawPose(queryIdx, params.interestingQueries, queryTable, ...
                                                    measurementTable, params.HoloLensViconSyncConstant);
    end
    rawPositions{i} = rawPosition;
    rawRotations{i} = rawRotation;
end

error = projectionError(queryInd, params.camera.origin.wrt.marker, params.camera.rotation.wrt.marker, ...
                        params.interestingPointsPC, params.interestingPointsQuery, ...
                        rawPositions, rawRotations, params);
for i=1:size(error)
    fprintf('Interesting query %d projection error: %0.2f\n', queryInd(i), error(i));
end
fprintf('Projection error sum: %0.2f\n', sum(error,1));

% translation / rotation error
errors = struct();
translationErrorSum = 0.0;
orientationErrorSum = 0.0;
nOptimalQueries = 0;
for i=1:nQueries
    queryIdx = queryInd(i);
    rawPosition = rawPositions{i};
    rawRotation = rawRotations{i};
    [R, t] = rawPoseToPose(rawPosition, rawRotation, params); 
    if ~isKey(params.optimal.camera.rotation.wrt.marker, queryIdx)
        continue;
    end
    if ~isKey(params.optimal.camera.origin.relative.wrt.marker, queryIdx)
        continue;
    end
    paramsOrig = params;
    params.camera.rotation.wrt.marker = params.optimal.camera.rotation.wrt.marker(queryIdx);
    params.camera.origin.relative.wrt.marker = params.optimal.camera.origin.relative.wrt.marker(queryIdx);
    params.camera.origin.wrt.marker = params.camera.originConstant * params.camera.origin.relative.wrt.marker;
    [refR, refT] = rawPoseToPose(rawPosition, rawRotation, params); 
    params = paramsOrig;

    translationError = norm(t - refT);
    orientationError = rotationDistance(refR, R);

    fprintf('Interesting query %d translation error: %0.2f, orientation error: %0.2f\n', ...
            queryIdx, translationError, orientationError);

    translationErrorSum = translationErrorSum + translationError;
    orientationErrorSum = orientationErrorSum + orientationError;
    nOptimalQueries = nOptimalQueries + 1;
end
fprintf('Average translation error:  %0.2f\n', translationErrorSum / nOptimalQueries);
fprintf('Average orientation error:  %0.2f\n', orientationErrorSum/ nOptimalQueries);
                                    
%% visualize correspondences and errors
for i=1:nQueries
    queryIdx = queryInd(i);
    rawPosition = rawPositions{i};
    rawRotation = rawRotations{i};
    projectedInterestingPoints = projectPoints(params.interestingPointsPC{queryIdx}, rawPosition, rawRotation, params);
    [R, t] = rawPoseToPose(rawPosition, rawRotation, params);
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
    set(gcf, 'Position', get(0, 'Screensize'));
end

end