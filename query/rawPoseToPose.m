function [R, t] = rawPoseToPose(rawPosition, rawRotation, params)
    viconOrigin = [-0.13; 0.04; 2.80]; % w.r.t. model
    viconRotation = deg2rad([90.0 180.0 0.0]); % w.r.t. model

    cameraRotation = deg2rad(params.camera.rotation.wrt.marker);
    
    % TODO: why does the rotation orientation have to be flipped?
    % NOTE: this is necessary for correctness
    % NOTE: this probably forces the ugly hack later on
    markerRotation = rawRotation .* [1.0 -1.0 1.0];

    cameraR = rotationMatrix(cameraRotation, 'XYZ'); % camera -> marker
    markerR = rotationMatrix(markerRotation, 'ZYX'); % marker -> vicon
    viconR = rotationMatrix(viconRotation, 'XYZ'); % vicon -> model

    markerOrigin = viconR * rawPosition + viconOrigin; % w.r.t. model

    % note: coordinate vectors are columns
    markerRwrtModel = viconR * markerR; % marker -> model
    
    cameraOrigin = markerRwrtModel * params.camera.origin.wrt.marker + markerOrigin; % w.r.t. model

    cameraRotation = markerR * cameraR; % w.r.t. vicon
    cameraRotation = viconR * cameraRotation; % w.r.t. model, camera points to y
    
    % bring z to where y is, as required by projectPC
    rFix = rotationMatrix([-pi/2, 0.0, 0.0], 'ZYX');
    cameraRotation = cameraRotation * rFix;

    t = cameraOrigin;
    R = cameraRotation;
    % TODO: I don't understand. suppose I have 3D vector x. Then I can rotate it by R*x. But if I want to "rotate"
    % it by cameraBases, I need to do inv(cameraBases)! But then why projectPoints works with R?
    % Is R actually a rotation matrix or a matrix with bases as columns??? Or both?
    % does it matter if I'm transforming a point versus another bases?
end