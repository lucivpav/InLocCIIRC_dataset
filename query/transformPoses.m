addpath('points2Image');
[ params ] = setupParams;

rawPosition = [-2.190593; -2.455224; 1.572377];
rawRotation = [-0.0056 0.0287 -1.4374]; % in radians

viconOrigin = [-1.886732; 0; 3.058354]; % w.r.t. model
viconOrigin = [0.1; 0; 3.058354]; % w.r.t. model
viconRotation = [90.0 180.0 0.0]; % in degrees, w.r.t. model
viconRotationRad = deg2rad(viconRotation);

cameraRotation = deg2rad([0.0 -7.0 5.73]); % w.r.t. marker
cameraR = rotationMatrix(cameraRotation, 'XYZ');

markerR = rotationMatrix(rawRotation, 'XYZ');
defaultMarkerDirection = [0.0; 1.0; 0.0];

viconR = rotationMatrix(viconRotationRad, 'XYZ');

markerOrigin = viconR * rawPosition + viconOrigin; % w.r.t. model

% note: coordinate vectors are columns
cameraCoordinateSystem = cameraR * eye(3); % w.r.t. marker
cameraCoordinateSystem = markerR * cameraCoordinateSystem; % w.r.t. vicon 
cameraCoordinateSystem = viconR * cameraCoordinateSystem; % w.r.t. model

cameraOrigin = 0.01 * [-3; 1; -4]; % w.r.t. camera, from marker origin
cameraOrigin = cameraCoordinateSystem * cameraOrigin + markerOrigin; % w.r.t. model, from model origin

pc = pcread(params.pointCloud.path);
f = 3172.435; % in pixels
sensorSize = [3024 4032];
outputSize = sensorSize;

R = cameraCoordinateSystem;

%%
R2 = rotationMatrix([-pi/2, 0.0, 0.0], 'ZYX'); % aka rFix, black magic
t = -cameraOrigin; % note the minus sign
projectedPointCloud = projectPointCloud(pc, f, R * R2, t, sensorSize, outputSize);
figure(1);
imshow(projectedPointCloud);