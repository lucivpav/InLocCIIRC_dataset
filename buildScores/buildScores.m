params = struct();
params.dataset.dir = '/home/lucivpav/InLocCIIRC_dataset';
params.inputs.dir = fullfile(params.dataset.dir, 'inputs');
params.input_features.dir = fullfile(params.dataset.dir, 'inputFeatures');
params.scores.path = fullfile(params.inputs.dir, 'scores.mat');
params.features.path = fullfile(params.input_features.dir, 'computed_features.mat');
params.cutouts.dir = fullfile(params.dataset.dir, 'cutouts');

files = dir(fullfile(params.cutouts.dir, '**/cutout*.jpg'));
nCutouts = size(files,1);

%x = matfile(params.features.path);
x = load(params.features.path);
queryFeatures = x.queryFeatures;
nQueries = size(queryFeatures,1);
score = zeros(nQueries, nCutouts);

for i=1:nQueries
    fprintf('processing query %d/%d\n', i, nQueries)
    thisQueryFeatures = queryFeatures(i,:);
    similarityScores = zeros(nCutouts,1);
    for j=1:nCutouts
        fprintf('loading cutout %d/%d\n', j, nCutouts)
        thisCutoutFeatures = x.cutoutFeatures(j,:);
        similarityScores(j,1) = dot(thisQueryFeatures, thisCutoutFeatures);
    end
    similarityScores = similarityScores / max(similarityScores); % softmax expects values in <0,1> range
    probabilityScores = softmax(similarityScores);
    score(i,:) = probabilityScores;
end

save(params.scores.path, 'score');