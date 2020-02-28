addpath('points2Image');
[ params ] = setupParams;

rawPosition = [-2.190593; -2.455224; 1.572377];
rawRotation = [-0.0056 0.0287 -1.4374]; % in radians

% the first step of this code is to transoform the marker's position
% and rotation into the model's coordinate system

viconOrigin = [-1.886732; 0; 3.058354]; % w.r.t. model coordinate system
viconRotation = [90.0 180.0 0.0]; % in degrees, w.r.t. model coordinate system

viconRotationRad = deg2rad(viconRotation);

r = rawRotation;
%R = angle2dcm(r(1), r(2), r(3), 'YXZ');
R = rotationMatrix(r(1), r(2), r(3), 'XYZ');
defaultMarkerDirection = [0.0; 1.0; 0.0];
markerDirection = R * defaultMarkerDirection; % w.r.t. vicon coordinate system

R = rotationMatrix(viconRotationRad(1),viconRotationRad(2),viconRotationRad(3), 'XYZ');

markerOrigin = R * rawPosition + viconOrigin; % w.r.t. model coordinate system
markerDirection = R * markerDirection; % w.r.t. model coordinate system

pc = pcread(params.pointCloud.path);
f = 3172.435; % in pixels
f = 2000.0;
sensorSize = [3024 4032];
outputSize = sensorSize;

P1 = [0.0; 0.0; 1.0];
P2 = markerDirection;
R = rotationMatrixBetweenTwoPoints(P1, P2);
xxx = R * P1;

% changing this should not influence the result (without rFix)
% but in fact it influences the result. WHY??
%%

% R = rotationMatrix(0.0, 0.0, r(3));
% zRotated = R * P1;
% 
% R = rotationMatrix(0.0, r(2), 0.0);
% yRotated = R * zRotated;
% 
% R = rotationMatrix(r(1), 0.0, 0.0);
% xRotated = R * yRotated;

%assert(isequal(xxx, xRotated)); % note: comparing floating points may fail

%%
rFix = [0.0, 0.0, 180.0]; % looks like I need  this shit
R2 = rotationMatrix(0.0, 0.0, pi, 'ZYX');
%r = r + rFix;
t = -markerOrigin; % note the minus sign
projectedPointCloud = projectPointCloud(pc, f, R * R2, t, sensorSize, outputSize);
figure(1);
imshow(projectedPointCloud);