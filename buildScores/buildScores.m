params = struct();
params.dataset.dir = '/datagrid/personal/lucivpav/InLocCIIRC_dataset_mirror';
params.inputs.dir = fullfile(params.dataset.dir, 'inputs');
params.input_features.dir = fullfile(params.dataset.dir, 'inputFeatures');
params.scores.path = fullfile(params.inputs.dir, 'scores.mat');
params.features.path = fullfile(params.input_features.dir, 'computed_features.mat');
params.cutouts.dir = fullfile(params.dataset.dir, 'cutouts');

files = dir(fullfile(params.cutouts.dir, '**/cutout*.jpg'));
nCutouts = size(files,1);

%x = matfile(params.features.path);
load(params.features.path, 'queryFeatures', 'cutoutFeatures');
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

save(params.scores.path, 'score');