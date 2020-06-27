function [modelToVicon, viconToMarker, markerToCamera, cameraToImage] = getModelToImageTransformations(markerOriginWrtVicon, ...
                                                                                                markerRotationWrtVicon, params)
    % markerToCamera: aka markerToEpsilon, i.e. columns are bases of marker wrt epsilon, see GVG
    % cameraToImage aka K, i.e. columns are beta bases wrt gamma
    identity = eye(4);
    cameraRotation = deg2rad(params.camera.rotation.wrt.marker);

    cameraR = rotationMatrix(cameraRotation, 'XYZ'); % camera -> marker
    markerR = rotationMatrix(markerRotationWrtVicon, 'XYZ'); % marker -> vicon
    viconR = rotationMatrix(params.vicon.rotation.wrt.model, 'XYZ'); % vicon -> model

    modelToVicon = identity;
    modelToVicon(1:3,1:3) = viconR;
    modelToVicon(1:3,4) = params.vicon.origin.wrt.model;
    modelToVicon = inv(modelToVicon);

    viconToMarker = identity;
    viconToMarker(1:3,1:3) = markerR;
    viconToMarker(1:3,4) = markerOriginWrtVicon;
    viconToMarker = inv(viconToMarker);

    markerToCamera = identity;
    markerToCamera(1:3,1:3) = cameraR;
    markerToCamera(1:3,4) = params.camera.origin.wrt.marker;
    markerToCamera = inv(markerToCamera);

    cameraToModel = inv(markerToCamera * viconToMarker * modelToVicon);
    cameraRotationWrtModel = cameraToModel(1:3,1:3);
    cameraOriginWrtModel = cameraToModel(1:3,4);

    cameraToImage = [params.K, zeros(3,1)];

    % Is R actually a rotation matrix or a matrix with bases as columns? Or both? -> Both
    % Does it matter if I'm transforming a point versus another bases? -> No
end