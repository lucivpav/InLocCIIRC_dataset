function projectedPoints = projectPoints(points, rawPosition, rawRotation, params)
    % points: 3xn matrix
    [R, t] = rawPoseToPose(rawPosition, rawRotation, params); 
    P = [params.camera.K*R, -params.camera.K*R*t];
    projectedPoints = projectPointsUsingP(points, P);
end