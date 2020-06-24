function [R, t] = rawPoseToPose(rawPosition, rawRotation, params)
    [modelToVicon, viconToMarker, markerToCamera, cameraToImage] = getModelToImageTransformations(rawPosition, ...
                                                                                                rawRotation, params);
    cameraToModel = inv(markerToCamera * viconToMarker * modelToVicon);
    cameraRotationWrtModel = cameraToModel(1:3,1:3);
    rFix = rotationMatrix([-pi/2, 0.0, 0.0], 'ZYX'); % bring z to where y is, as required by projectPC
    R = cameraRotationWrtModel * rFix; % wrt model, suitable for projection
    t = cameraToModel(1:3,4); % wrt model

end