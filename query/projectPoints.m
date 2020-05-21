function projectedPoints = projectPoints(points, rawPosition, rawRotation, params)
    % points: 3xn matrix

    [R, t] = rawPoseToPose(rawPosition, rawRotation, params); 
    
    % PointCloud <-> query correspondences
    nCorrespondences = size(points,2);
    
    P = [params.K*R, -params.K*R*t];
    toProject = [points; ones(1,nCorrespondences)];
    projectedPoints = P * toProject;
    projectedPoints = projectedPoints ./ projectedPoints(3,:);
    projectedPoints = projectedPoints(1:2,:);

end