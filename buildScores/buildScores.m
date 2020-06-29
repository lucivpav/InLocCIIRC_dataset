addpath('../functions/InLocCIIRC_utils/params');

params = setupParams('s10e'); % NOTE: mode can be anything, its specific params are not used here

featuresPath = fullfile(params.input.feature.dir, 'computed_features.mat');

files = dir(fullfile(params.dataset.db.cutouts.dir, '**/cutout*.jpg'));
nCutouts = size(files,1);

%x = matfile(featuresPath);
load(featuresPath, 'queryFeatures', 'cutoutFeatures');
nQueries = size(queryFeatures,1);
score = zeros(nQueries, nCutouts, 'single');

cutoutFeatures = cutoutFeatures';

for i=1:nQueries
    fprintf('processing query %d/%d\n', i, nQueries)
    thisQueryFeatures = queryFeatures(i,:);
    thisQueryFeatures = repmat(thisQueryFeatures, nCutouts, 1)';
    similarityScores = dot(thisQueryFeatures, cutoutFeatures);
    similarityScores = similarityScores / max(similarityScores); % softmax expects values in <0,1> range
    probabilityScores = softmax(similarityScores);
    score(i,:) = probabilityScores;
end

save(params.input.scores.path, 'score');