panoId = 20;
panoProjectionIdx = -1; % -1 for the first step, valid idx for the second step

load(params.sweepData.mat.path);
pc = pcread(params.pointCloud.path);

fun = @(x) sweepData(x).panoId == panoId;
tf = arrayfun(fun, 1:numel(sweepData));
sweepRecord = sweepData(find(tf));

if exist(params.temporary.dir, 'dir') ~= 7
    mkdir(params.temporary.dir);
end

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
panoImg = imread(fullfile(params.panoramas.dir, strcat(int2str(panoId), '.jpg')));
viewSize = outputSize(1);
fov = 2*atan((sensorSize(1)/2)/f);
nViews = 256;
panoramaProjections = projectPanorama(panoImg, viewSize, fov, nViews);
%plotMany(panoramaProjections);

if (panoProjectionIdx ~= -1)
    xMid = ((panoramaProjections(panoProjectionIdx).vx + pi) / (2*pi)) * size(panoImg, 2);
    xMid = round(xMid);
    rmdir(params.temporary.dir);
    %% Perform panorama rotation and save the result
    panoMid = round(size(panoImg, 2)/2);
    rotatedPanorama = [panoImg(:,xMid:end,:) panoImg(:,1:xMid,:)];
    rotatedPanorama = [rotatedPanorama(:,panoMid:end,:) rotatedPanorama(:,1:panoMid,:)];
    %figure(4);
    %imshow(rotatedPanorama);
    imwrite(rotatedPanorama, fullfile(params.rotatedPanoramas.dir, strcat(int2str(panoId), '.jpg')));
    exit();
end

for idx=1:size(panoramaProjections, 2)
    panoramaProjection = panoramaProjections(idx).img;
    imshowpair(projectedPointCloud, panoramaProjection, 'montage');
    saveas(gcf, fullfile(params.temporary.dir, strcat(int2str(idx), '.jpg')));
end