function error = projectionError(queryInd, origin, rotation, interestingPointsPC, interestingPointsQuery, ...
                                    rawPositions, rawRotations, params)
    nQueries = size(queryInd,2);
    error = zeros(nQueries,1);
    for i=1:nQueries
        queryIdx = queryInd(i);

        paramsBak = params;
        params.camera.origin.wrt.marker = origin;
        params.camera.rotation.wrt.marker = rotation;
        projectedInterestingPoints = projectPoints(interestingPointsPC{queryIdx}, rawPositions{i}, rawRotations{i}, params);
        params = paramsBak;

        thisInterestingPointsQuery = interestingPointsQuery{queryIdx};
        error(i) = norm(thisInterestingPointsQuery - projectedInterestingPoints);
    end
end