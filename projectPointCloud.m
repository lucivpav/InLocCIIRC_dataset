pc = pcread('matterpak_sehs6V3VnSW/cloud - rotated.ply');
intensities = single(pc.Color(:,1:3))/255;
points = [pc.Location(:,1:3), intensities];
f = 10;
px = 300;
py = 300;
K = [f, 0, px; 0, f, py; 0, 0, 1];
r = [0.17682930550100803, 3.651460591341834, -0.11934799025763472];
R = eul2rotm(r);
t = [0.0009786105947569013; 1.6932588815689087; 0.06866297125816345];
P = K * [R, t];
imageSize = [1000,1000];

[projected, valid] = projectPoints(points, P, [], [], imageSize, false);
projected = projected(valid,:);

scatter(projected(:,1),projected(:,2),20,projected(:,3:5),'fill');
axis equal;
title('Points projected with camera model');