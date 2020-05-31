function evaluateMatches(queryInd, params, queryTable, measurementTable)

for i=1:size(queryInd,2)
    queryIdx = queryInd(i);
    [rawPosition, rawRotation] = buildRawPose(queryIdx, params.interestingQueries, queryTable, ...
                                                measurementTable, params.HoloLensViconSyncConstant);
    rawPositions{i} = rawPosition;
    rawRotations{i} = rawRotation;
end

error = projectionError(queryInd, params.camera.origin.wrt.marker, params.camera.rotation.wrt.marker, ...
                        params.interestingPointsPC, params.interestingPointsQuery, ...
                        rawPositions, rawRotations, params);
for i=1:size(error)
    fprintf('Interesting query %d error: %0.2f\n', queryInd(i), error(i));
end
fprintf('Error sum: %0.2f\n', sum(error,1));
                                    
%% visualize correspondences and errors
for i=1:size(queryInd,2)
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