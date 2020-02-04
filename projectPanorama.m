% read image
panoImg = imread('./panoramas/2.jpg');
panoImg = im2double(panoImg);

% project it to multiple perspective views
cutSize = 300; % size of perspective views

nAngles = 128; % TODO: rename
% TODO: find a direct conversion from focal length in projectPointCloud
fov = 1.58; % horizontal field of view of perspective views
xh = -pi:(pi/nAngles):((nAngles-1)/nAngles*pi);
%shift = 1.02;
%xh = 0+shift:(pi/16):(15/16*pi+shift);
yh = zeros(1, length(xh));
x = [xh];
y = [yh]; % viewing direction of perspective views

[sepScene] = separatePano( panoImg, fov, x, y, cutSize);

%% visualization: random display 9 perspective views
nViews = nAngles*2;
%ID = randsample(length(sepScene), nViews);
ID = 1:length(sepScene);

figure(1);
uv = [x(ID)' y(ID)'];
coords = uv2coords(uv, 1024, 512);
% imshow(imresize(panoImg, [512, 1024])); hold on
% for i = 1:nViews
%     scatter(coords(i,1),coords(i,2), 40, [1 0 0],'fill');
%     text(coords(i,1)+8, coords(i,2),sprintf('%d', i), ...
%         'BackgroundColor',[.7 .9 .7], ...
%         'Color', [1 0 0]); 
% end
% title('Project viewpoints to perspective views: Image');

figure(2);
for i = 1:nViews
    subplot(16,16,i);
    imshow(sepScene(ID(i)).img);
    title(sprintf('%d: (%3.2f, %3.2f)', ...
        i, sepScene(ID(i)).vx, sepScene(ID(i)).vy));
end

for i = 1:nViews
    projectedPanoramas(i).img = sepScene(i).img;
end
save('projectedPanoramas/2.mat', 'projectedPanoramas', '-v7.3');