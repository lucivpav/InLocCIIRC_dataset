localIds = [2 3 4 5 6 7 8 9 10 11 12 13 15, ...
            16 20 21 22 23 24 25 26 27 28 29 30 31, ...
            32 33 35 36 37];
localIds = [11];

load('sweepData.mat');
pc = pcread('matterpak_sehs6V3VnSW/cloud - rotated.ply');

for i=1:size(localIds,2)
localId = localIds(i);
fun = @(x) sweepData(x).localId == localId;
tf = arrayfun(fun, 1:numel(sweepData));
sweepRecord = sweepData(find(tf));

%% Project the point cloud
f = 500;
sensorSize = [1000,1000];
rFix = [0.0, 0.0, 180.0];
r = rFix + sweepRecord.rotation;
t = -sweepRecord.position;
outputSize = [300 300];
projectedPointCloud = projectPointCloud(pc, f, r, t, sensorSize, outputSize);
%figure(1);
%imshow(projectedPointCloud);

%% Project the panorama
panoImg = imread(sprintf('./panoramas/%d.jpg', localId));
viewSize = outputSize(1);
fov = 2*atan((sensorSize(1)/2)/f);
nViews = 256;
panoramaProjections = projectPanorama(panoImg, viewSize, fov, nViews);
%save(sprintf('projectedPanoramas/%d.mat', panorama), 'panoramaProjections', '-v7.3');
%plotMany(panoramaProjections);

%% Find best rotation
goodness = sweepRecord.goodness;
[xMid, panorama, panoIdx, diffs] = findRotation(projectedPointCloud, panoramaProjections, panoImg, goodness);
sprintf('Found match: panorama #%d', panoIdx)
figure(3);
imshowpair(projectedPointCloud, panorama, 'montage');
saveas(gcf, sprintf('panoramas2pointClouds/%d.jpg', localId), 'jpg');

%% Perform panorama rotation and save the result
panoMid = round(size(panoImg, 2)/2);
rotatedPanorama = [panoImg(:,xMid:end,:) panoImg(:,1:xMid,:)];
rotatedPanorama = [rotatedPanorama(:,panoMid:end,:) rotatedPanorama(:,1:panoMid,:)];
%figure(4);
%imshow(rotatedPanorama);
imwrite(rotatedPanorama, sprintf('rotatedPanoramas/%d.jpg', localId));

% verification
%rotatedPanorama = im2double(rotatedPanorama);
%[sepScene] = separatePano(rotatedPanorama, fov, [0.0], [0.0], viewSize);
%figure(5);
%imshow(sepScene(1).img);
end