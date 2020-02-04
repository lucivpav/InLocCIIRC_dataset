panorama = '2';

%% Project the point cloud
pc = pcread('matterpak_sehs6V3VnSW/cloud - rotated.ply');
f = 500;
rFix = [0.0, 0.0, 180.0];
%r = rFix + [0.17682930550100803, 3.651460591341834, -0.11934799025763472];
r = rFix + [0.0, 3.651460591341834, 1.0]; % TODO: understand this hack!
t = [0.0009786105947569013; 1.6932588815689087; 0.06866297125816345];
outputSize = [300 300];
projectedPointCloud = projectPointCloud(pc, f, r, t, outputSize);

%% Project the panorama
panoImg = imread(sprintf('./panoramas/%s.jpg', panorama));
viewSize = outputSize(1);
% TODO: find a direct conversion from focal length f
fov = 1.58;
nViews = 256;
panoramaProjections = projectPanorama(panoImg, viewSize, fov, nViews);
%save(sprintf('projectedPanoramas/%d.mat', panorama), 'panoramaProjections', '-v7.3');
%plotMany(panoramaProjections);

%% Find best rotation
[xMid, panorama, bestIdx, diffs] = findRotation(projectedPointCloud, panoramaProjections);
sprintf('Best match: panorama #%d', bestIdx)
figure(3);
imshowpair(projectedPointCloud, panorama, 'montage');