pointCloudPerspective = rgb2gray(imread('pointCloudPerspective.jpg'));

load('projectedPanoramas/2.mat');

nCandidates = size(projectedPanoramas, 2);
losses = zeros([nCandidates,1]);
se = offsetstrel('ball', 3, 3);
edge1 = uint8(255 * edge(pointCloudPerspective));
dilated1 = imdilate(edge1, se);

for i=1:nCandidates
    panoramaPerspective = rgb2gray(projectedPanoramas(i).img); 
    edge2 = uint8(255 * edge(panoramaPerspective));
    dilated2 = imdilate(edge2, se);
    diff(i).img = bitxor(dilated1, dilated2);
end

% figure(2);
% for i = 1:nCandidates
%     subplot(16,16,i);
%     imshow(diff(i).img);
%     title(sprintf('%d', i));
% end

for i=1:nCandidates
    losses(i,1) = sum(diff(i).img(:)==255);
end

[sorted, idx] = sort(losses);

bestIdx = idx(1);
sprintf('Best match: panorama #%d', bestIdx)
figure(3);
reference = imread('pointCloudPerspective.jpg');
result = projectedPanoramas(bestIdx).img;
imshowpair(reference, result, 'montage');