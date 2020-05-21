function error = projectionError(queryInd, origin, rotation, interestingPointsPC, interestingPointsQuery, ...
                                    rawPositions, rawRotations, params)
    error = 0.0;
    for i=1:size(queryInd,2)
        queryIdx = queryInd(i);

        paramsBak = params;
        params.camera.origin.wrt.marker = origin;
        params.camera.rotation.wrt.marker = rotation;
        projectedInterestingPoints = projectPoints(interestingPointsPC{queryIdx}, rawPositions{i}, rawRotations{i}, params);
        params = paramsBak;

        thisInterestingPointsQuery = interestingPointsQuery{queryIdx};
        error = error + norm(thisInterestingPointsQuery - projectedInterestingPoints);
    end
end