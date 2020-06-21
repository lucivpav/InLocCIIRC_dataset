function [R, t] = rawPoseToPose(rawPosition, rawRotation, params)
    viconOrigin = [-0.13; 0.04; 2.80]; % w.r.t. model
    viconRotation = deg2rad([90.0 180.0 0.0]); % w.r.t. model % NOTE: 'XYZ' order
    %viconRotation = deg2rad([-82.0 -6.0 176.0]); % w.r.t. model

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

    cameraRotation = markerR * cameraR; % w.r.t. vicon % seems like this fix is NOT wrong!
    cameraRotation = params.vicon.to.model.pre.rotation.matrix * cameraRotation; % wrt magic CS
    %cameraRotation = cameraR * markerR; % w.r.t. vicon % ORIGINAL % this is WRONG, as demonstrated in debugCS.py Why?
                                         % because here I am rotation AROUND THE VICON's Z-axis (not the marker Z-axis)
    cameraRotation = viconR * cameraRotation; % w.r.t. model, camera points to y
    
    % bring z to where y is, as required by projectPC
    rFix = rotationMatrix([-pi/2, 0.0, 0.0], 'ZYX');
    cameraRotation = cameraRotation * rFix;

    t = cameraOrigin;
    R = cameraRotation;
    % Why projectPoints works with R? -> Because 
    % in P = [params.K*R, -params.K*R*t], we want to go the oposite way: from world coordinates to camera coordinates
    % Is R actually a rotation matrix or a matrix with bases as columns? Or both? -> Both
    % Does it matter if I'm transforming a point versus another bases? -> No
end