function projectedPointCloud = projectPointCloud(pc, f, r, t, imageSize, outputSize)
%PROJECTPOINTCLOUD generate a perspective image from a point cloud
% as taken by a camera
% pc: point cloud
% f: focal length
% r: 3D rotation as a 1x3 vector
% t: 3D translation as a 3x1 vector
% imageSize: size of the camera sensor as 1x2 vector
% outputSize: size of the resulting 2D image as a 1x2 vector

intensities = single(pc.Color(:,1:3))/255;
points = [pc.Location(:,1:3), intensities];

angles = r*pi/180;
T = eye(4);
T(1:3,4) = t;

px = imageSize(1)/2;
py = imageSize(2)/2;

K = eye(3);
K(1,1) = f;
K(2,2) = f;
K(1,3) = px;
K(2,3) = py;
R = angle2dcm(angles(1),angles(2),angles(3));
cam = K * [R [0.0; 0.0; 0.0]];

[ image ] = points2Image(points, imageSize, cam, T, 5, 1);

projectedPointCloud = imresize(image, outputSize);