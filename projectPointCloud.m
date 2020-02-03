pc = pcread('matterpak_sehs6V3VnSW/cloud - rotated.ply');
intensities = single(pc.Color(:,1:3))/255;
points = [pc.Location(:,1:3), intensities];
f = 500;
rFix = [0.0, 0.0, 180.0];
r = rFix + [0.17682930550100803, 3.651460591341834, -0.11934799025763472];
t = [0.0009786105947569013; 1.6932588815689087; 0.06866297125816345];
imageSize = [1000,1000];

angles = r*pi/180;
position = t;
T = eye(4);
T(1:3,1:3) = angle2dcm(angles(1),angles(2),angles(3));
T(1:3,4) = position;

[ image ] = points2Image(points, imageSize, f, T, 10, 1);
imshow(image);