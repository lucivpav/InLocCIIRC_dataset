addpath('../functions/relja_matlab');
addpath('../functions/relja_matlab/matconvnet/');
addpath('../functions/netvlad/');
addpath('../functions/InLocCIIRC_utils/at_netvlad_function');
run('../functions/matconvnet/matlab/vl_setupnn.m');

params = struct();
params.dataset.dir = '/datagrid/personal/lucivpav/InLocCIIRC_dataset';
params.data.netvlad.pretrained = fullfile(params.dataset.dir, 'NetVLAD', 'vd16_pitts30k_conv5_3_vlad_preL2_intra_white.mat');
params.inputs.dir = fullfile(params.dataset.dir, 'inputs');
params.input_features.dir = fullfile(params.dataset.dir, 'inputFeatures');
params.query_imgnames_all.path = fullfile(params.inputs.dir, 'query_imgnames_all.mat');
params.cutout_imgnames_all.path = fullfile(params.inputs.dir, 'cutout_imgnames_all.mat');
params.query.dir = fullfile(params.dataset.dir, 'query-s10e/'); % NOTE: the slash is important
params.cutouts.dir = fullfile(params.dataset.dir, 'cutouts/');

x = load(params.cutout_imgnames_all.path);
cutoutImageFilenames = x.cutout_imgnames_all;
cutoutSize = size(imread(fullfile(params.cutouts.dir, cutoutImageFilenames{1})));
cutoutSize = [cutoutSize(2), cutoutSize(1)]; % width, height

if exist(params.input_features.dir, 'dir') ~= 7
    mkdir(params.input_features.dir);
end

load(params.data.netvlad.pretrained, 'net');
net = relja_simplenn_tidy(net);
net = relja_cropToLayer(net, 'preL2');

%% query
x = load(params.query_imgnames_all.path);
queryImageFilenames = x.query_imgnames_all;

featureLength = 3840000;

%serialAllFeats(net, params.query.dir, queryImageFilenames, params.input_features.dir, 'useGPU', false, 'batchSize', 1);

nQueries = size(queryImageFilenames,2);
queryFeatures = zeros(nQueries, featureLength, 'single');
for i=1:nQueries
    fprintf('Finding features for query #%d/%d\n\n', i, nQueries)
    queryImage = load_query_image_compatible_with_cutouts(fullfile(params.query.dir, queryImageFilenames{i}), cutoutSize);
    cnn = at_serialAllFeats_convfeat(net, queryImage, 'useGPU', true);
    queryFeatures(i,:) = cnn{5}.x(:);
end

%% cutouts
nCutouts = size(cutoutImageFilenames,2);
cutoutFeatures = zeros(nCutouts, featureLength, 'single');
for i=1:nCutouts
    fprintf('Finding features for cutout #%d/%d\n\n', i, nCutouts)
    cutoutImage = imread(fullfile(params.cutouts.dir, cutoutImageFilenames{i}));
    cnn = at_serialAllFeats_convfeat(net, cutoutImage, 'useGPU', true);
    cutoutFeatures(i,:) = cnn{5}.x(:);
end

%% save the features
p = fullfile(params.input_features.dir, 'computed_features.mat');
save(p, 'queryFeatures', 'cutoutFeatures', '-v7.3');