addpath('../functions/relja_matlab');
addpath('../functions/relja_matlab/matconvnet/');
addpath('../functions/netvlad/');
addpath('../functions/at_netvlad_function');
run('../functions/matconvnet/matlab/vl_setupnn.m');

params = struct();
params.dataset.dir = '/datagrid/personal/lucivpav/InLocCIIRC_dataset';
params.data.netvlad.pretrained = fullfile(params.dataset.dir, 'NetVLAD', 'vd16_pitts30k_conv5_3_vlad_preL2_intra_white.mat');
params.inputs.dir = fullfile(params.dataset.dir, 'inputs');
params.input_features.dir = fullfile(params.dataset.dir, 'inputFeatures');
params.query_imgnames_all.path = fullfile(params.inputs.dir, 'query_imgnames_all.mat');
params.cutout_imgnames_all.path = fullfile(params.inputs.dir, 'cutout_imgnames_all.mat');
params.query.dir = fullfile(params.dataset.dir, 'query/'); % NOTE: the slash is important
params.cutouts.dir = fullfile(params.dataset.dir, 'cutouts/');

if exist(params.input_features.dir, 'dir') ~= 7
    mkdir(params.input_features.dir);
end

load(params.data.netvlad.pretrained, 'net');
net = relja_simplenn_tidy(net);
net = relja_cropToLayer(net, 'preL2');

%% query
x = load(params.query_imgnames_all.path);
imageFilenames = x.query_imgnames_all;

%serialAllFeats(net, params.query.dir, imageFilenames, params.input_features.dir, 'useGPU', false, 'batchSize', 1);
for i=1:size(imageFilenames,2)
    fprintf('Finding features for query #%d/%d\n\n', i, size(imageFilenames,2))
    cnn = at_serialAllFeats_convfeat(net, params.query.dir, imageFilenames{i}, 'useGPU', true);
    queryFeatures(i,:) = cnn{5}.x(:);
end

%% cutouts
x = load(params.cutout_imgnames_all.path);
imageFilenames = x.cutout_imgnames_all;
for i=1:size(imageFilenames,2)
    fprintf('Finding features for cutout #%d/%d\n\n', i, size(imageFilenames,2))
    cnn = at_serialAllFeats_convfeat(net, params.cutouts.dir, imageFilenames{i}, 'useGPU', true);
    cutoutFeatures(i,:) = cnn{5}.x(:);
end

%% save the features
p = fullfile(params.input_features.dir, 'computed_features.mat');
save(p, 'queryFeatures', 'cutoutFeatures', '-v7.3');