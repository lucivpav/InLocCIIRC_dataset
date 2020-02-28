function projectedPointCloud = projectPointCloud(pc, f, R, t, sensorSize, outputSize)
%PROJECTPOINTCLOUD generate a perspective image from a point cloud
% as taken by a camera
% pc: point cloud
% f: focal length
% R: rotation matrix
% t: 3D translation as a 3x1 vector
% sensorSize: size of the camera sensor as 1x2 vector
% outputSize: size of the resulting 2D image as a 1x2 vector

intensities = single(pc.Color(:,1:3))/255;
points = [pc.Location(:,1:3), intensities];

T = eye(4);
T(1:3,4) = t;

px = sensorSize(1)/2;
py = sensorSize(2)/2;

K = eye(3);
K(1,1) = f;
K(2,2) = f;
K(1,3) = px;
K(2,3) = py;
cam = K * [R [0.0; 0.0; 0.0]];

[ image ] = points2Image(points, sensorSize, cam, T, 6, 1);

projectedPointCloud = imresize(image, outputSize);