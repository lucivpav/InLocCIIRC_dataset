params = struct();
params.dataset.dir = '/Volumes/GoogleDrive/Můj disk/ARTwin/InLocCIIRC_dataset'; % Note: this path is intentionally not UTF-8 style. This is because dir() cannot handle UTF-8 characters properly.
params.inputs.dir = fullfile(params.dataset.dir, 'inputs');
params.query.dir = fullfile(params.dataset.dir, 'query-HoloLens1');
params.query_imgnames_all.path = fullfile(params.inputs.dir, 'query_imgnames_all.mat');
params.cutouts.dir = fullfile(params.dataset.dir, 'cutouts');
params.cutout_imgnames_all.path = fullfile(params.inputs.dir, 'cutout_imgnames_all.mat');

if exist(params.inputs.dir, 'dir') ~= 7
    mkdir(params.inputs.dir);
end

%% query
files = dir(fullfile(params.query.dir, '*.jpg'));
nFiles = size(files,1);
query_imgnames_all = cell(1,nFiles);

for i=1:nFiles
    query_imgnames_all{1,i} = files(i).name;
end

save(params.query_imgnames_all.path, 'query_imgnames_all');

%% cutouts
files = dir(fullfile(params.cutouts.dir, '**/cutout*.jpg'));
nFiles = size(files,1);
cutout_imgnames_all = cell(1,nFiles);

for i=1:nFiles
    relativePath = extractAfter(files(i).folder, size(params.cutouts.dir,2)+1);
    cutout_imgnames_all{1,i} = fullfile(relativePath, files(i).name);
end

save(params.cutout_imgnames_all.path, 'cutout_imgnames_all');