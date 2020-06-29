function [projectionErrorSum, meanTranslationError, meanOrientationError] = evaluateMatches(queryInd, ...
                                                            params, queryTable, measurementTable, rawPosesTable)
    % either queryTable, measurementTable are set, or
    % rawPosesTable is set
    % those values that are not set, should be set to false

addpath('../functions/InLocCIIRC_utils/rotationDistance');
addpath('../functions/InLocCIIRC_utils/projectPointCloud');

nQueries = size(queryInd,2);
for i=1:nQueries
    queryIdx = queryInd(i);

    [rawPosition, rawRotation] = getRawPose(queryIdx, params.interestingQueries, queryTable, ...
                                            measurementTable, rawPosesTable, params);

    rawPositions{i} = rawPosition;
    rawRotations{i} = rawRotation;
end

error = projectionError(queryInd, params.camera.origin.wrt.marker, params.camera.rotation.wrt.marker, ...
                        params.interestingPointsPC, params.interestingPointsQuery, ...
                        rawPositions, rawRotations, params);
for i=1:size(error)
    queryIdx = queryInd(i);
    nCorrespondences = size(params.interestingPointsPC{queryIdx},2);
    fprintf('Interesting query %d average projection error: %0.2f, sum: %0.2f\n', queryInd(i), ...
            error(i)/nCorrespondences, error(i));
end
projectionErrorSum = sum(error,1);
fprintf('Projection error sum: %0.2f\n', projectionErrorSum);

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
    if iscell(params.camera.rotation.wrt.marker) % because MATLAB sucks
        params.camera.rotation.wrt.marker = params.camera.rotation.wrt.marker{1};
    end
    params.camera.origin.relative.wrt.marker = params.optimal.camera.origin.relative.wrt.marker(queryIdx);
    if iscell(params.camera.origin.relative.wrt.marker)
        params.camera.origin.relative.wrt.marker = params.camera.origin.relative.wrt.marker{1};
    end
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
meanTranslationError = translationErrorSum / nOptimalQueries;
meanOrientationError = orientationErrorSum / nOptimalQueries;
fprintf('Average translation error:  %0.2f\n', meanTranslationError);
fprintf('Average orientation error:  %0.2f\n', meanOrientationError);

env = environment();

if ~strcmp(env, 'laptop')
    return;
end
                                    
%% visualize correspondences and errors
%return;
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
                                        params.projectPointCloudPy.path); % TODO: use projectMesh instead, which can work in headless mode
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
