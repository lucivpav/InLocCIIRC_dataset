function [R, t] = rawPoseToPose(rawPosition, rawRotation, params)
    [modelToVicon, viconToMarker, markerToCamera, cameraToImage] = getModelToImageTransformations(rawPosition, ...
                                                                                                rawRotation, params);
    modelToCamera = markerToCamera * viconToMarker * modelToVicon;
    cameraToModel = inv(modelToCamera);
    R = modelToCamera(1:3,1:3); % columns are bases of model wrt epsilon (see GVG)
    t = cameraToModel(1:3,4); % wrt model

end