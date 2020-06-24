function [modelToVicon, viconToMarker, markerToCamera, cameraToImage] = getModelToImageTransformations(markerOriginWrtVicon, ...
                                                                                                markerRotationWrtVicon, params)
    identity = eye(4);
    cameraRotation = deg2rad(params.camera.rotation.wrt.marker);

    % TODO: why does the rotation orientation have to be flipped?
    % NOTE: this is necessary for correctness
    % NOTE: this probably forces the ugly hack later on
    markerRotation = markerRotationWrtVicon .* [1.0 -1.0 1.0];

    cameraR = rotationMatrix(cameraRotation, 'XYZ'); % camera -> marker
    markerR = rotationMatrix(markerRotation, 'ZYX'); % marker -> vicon
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

    rFix = rotationMatrix([-pi/2, 0.0, 0.0], 'ZYX'); % bring z to where y is, as required by projectPC
    cameraRotationWrtModelForProjection = cameraRotationWrtModel * rFix;
    modelToImage = [params.K*cameraRotationWrtModelForProjection, ...
                        -params.K*cameraRotationWrtModelForProjection*cameraOriginWrtModel];
    cameraToImage = modelToImage * cameraToModel; % TODO: compute this without using modelToImage, that will save LOC

    % Why projectPoints works with R? -> Because
    % in P = [params.K*R, -params.K*R*t], we want to go the oposite way: from world coordinates to image coordinates
    % Is R actually a rotation matrix or a matrix with bases as columns? Or both? -> Both
    % Does it matter if I'm transforming a point versus another bases? -> No
end