addpath('../functions/InLocCIIRC_utils/params');

params = setupParams('holoLens1'); % NOTE: mode can be anything, its specific params are not used here

featuresPath = fullfile(params.input.feature.dir, 'computed_features.mat');

files = dir(fullfile(params.dataset.db.cutouts.dir, '**/cutout*.jpg'));
nCutouts = size(files,1);

%x = matfile(featuresPath);
load(featuresPath, 'queryFeatures', 'cutoutFeatures');
nQueries = size(queryFeatures,1);
score = zeros(nQueries, nCutouts, 'single');

cutoutFeatures = cutoutFeatures';

tol = 1e-6;
if ~all(abs(vecnorm(cutoutFeatures)-1.0)<tol)
    fprintf('norm: %f\n', vecnorm(cutoutFeatures));
    error('Features are not normalized!');
end
for i=1:nQueries
    fprintf('processing query %d/%d\n', i, nQueries);
    thisQueryFeatures = queryFeatures(i,:);
    if ~all(abs(norm(thisQueryFeatures)-1.0)<tol)
        fprintf('norm: %f\n', norm(thisQueryFeatures));
        error('Features are not normalized!');
    end
    thisQueryFeatures = repmat(thisQueryFeatures, nCutouts, 1)';
    similarityScores = dot(thisQueryFeatures, cutoutFeatures);
    score(i,:) = similarityScores; % NOTE: this is not a probability distribution (and it does not have to be)
end

save(params.input.scores.path, 'score');