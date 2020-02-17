panoIds = [2 3 4 5 6 7 8 9 10 11 12 13 15, ...
            16 20 21 22 23 24 25 26 27 28 29 30 31, ...
            32 33 35 36 37];
panoIds = [2 3];

cutoutsDir = 'cutouts';
f = 600;
sensorSize = [1600 1200];
fov = 2*atan((sensorSize(2)/2)/f);

load('sweepData.mat');

for i=1:size(panoIds,2)
panoId = panoIds(i);
fun = @(x) sweepData(x).panoId == panoId;
tf = arrayfun(fun, 1:numel(sweepData));
sweepRecord = sweepData(find(tf));

thisPanoCutoutsDir = fullfile(cutoutsDir, int2str(panoId));
if exist(thisPanoCutoutsDir, 'dir') ~= 7
    mkdir(thisPanoCutoutsDir);
end

panoImg = imread(sprintf('./rotatedPanoramas/%d.jpg', panoId));
panoImg = im2double(panoImg);

xh = -180:30:180-1;
yh0 = zeros(1, length(xh));
yhTop = yh0 + 30;
yhBottom = yh0 - 30;

xDeg = [xh xh xh];
yDeg = [yh0 yhTop yhBottom];

x = deg2rad(xDeg);
y = deg2rad(yDeg);

[sepScene] = separatePano(panoImg, fov, x, y, sensorSize);

%% visualization
nViews = length(sepScene);
ID = randsample(length(sepScene), nViews);

figure(1);
uv = [x(ID)' y(ID)'];
coords = uv2coords(uv, 1024, 512);
imshow(imresize(panoImg, [512, 1024])); hold on
for i = 1:nViews
    scatter(coords(i,1),coords(i,2), 40, [1 0 0],'fill');
    text(coords(i,1)+8, coords(i,2),sprintf('%d', i), ...
        'BackgroundColor',[.7 .9 .7], ...
        'Color', [1 0 0]); 
end
title('Project viewpoints to perspective views: Image');
%%
for i = 1:nViews
    yaw = xDeg(i);
    pitch = yDeg(i);
    filename = sprintf('cutout_%d_%d_%d.jpg', panoId, yaw, pitch);
    filepath = fullfile(thisPanoCutoutsDir, filename);
    imwrite(sepScene(i).img, filepath);
    filename = sprintf('%s.mat', filename);
    filepath = fullfile(thisPanoCutoutsDir, filename);
    RGBcut = im2uint8(sepScene(i).img);
    cameraPosition = sweepRecord.position;
    cameraRotation = (sweepRecord.rotation + [pitch yaw 0.0])';
    focalLength = f;
    save(filepath, 'RGBcut', 'cameraPosition', 'cameraRotation', 'focalLength', 'sensorSize');
end
end