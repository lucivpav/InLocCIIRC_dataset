function [R, t] = rawPoseToPose(rawPosition, rawRotation, params)
    viconOrigin = [-0.13; 0.04; 2.80]; % w.r.t. model
    viconRotation = deg2rad([90.0 180.0 0.0]); % w.r.t. model

    cameraRotation = params.camera.rotation.wrt.marker;
    
    % TODO: why does the rotation orientation have to be flipped?
    % NOTE: this is necessary for correctness
    % NOTE: this probably forces the ugly hack later on
    markerRotation = rawRotation .* [1.0 -1.0 1.0];

    cameraR = rotationMatrix(cameraRotation, 'XYZ');
    markerR = rotationMatrix(markerRotation, 'ZYX');
    viconR = rotationMatrix(viconRotation, 'XYZ');

    markerOrigin = viconR * rawPosition + viconOrigin; % w.r.t. model

    % note: coordinate vectors are columns
    markerCoordinateSystem = markerR * eye(3); % w.r.t. vicon
    markerCoordinateSystem = viconR * markerCoordinateSystem; % w.r.t. model
    
    cameraOrigin = params.camera.origin.wrt.marker;
    cameraOrigin = markerCoordinateSystem * cameraOrigin + markerOrigin; % w.r.t. marker, from model origin

    markerCoordinateSystem = markerR * eye(3); % w.r.t. vicon
    cameraCoordinateSystem = cameraR * markerCoordinateSystem; % w.r.t. vicon 
    cameraCoordinateSystem = viconR * cameraCoordinateSystem; % w.r.t. model, camera points to y
    
    % bring z to where y is, as required by projectPC
    rFix = rotationMatrix([-pi/2, 0.0, 0.0], 'ZYX');
    cameraCoordinateSystem = cameraCoordinateSystem * rFix;

    t = cameraOrigin;
    R = cameraCoordinateSystem;
end