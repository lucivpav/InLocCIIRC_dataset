addpath('../functions/InLocCIIRC_utils/params');

params = setupParams('s10e'); % TODO: adjust

if exist(params.input.dir, 'dir') ~= 7
    mkdir(params.input.dir);
end

%% query
files = dir(fullfile(params.dataset.query.dir, '*.jpg'));
nFiles = size(files,1);
query_imgnames_all = cell(1,nFiles);

for i=1:nFiles
    query_imgnames_all{1,i} = files(i).name;
end

save(params.input.qlist.path);

%% cutouts
files = dir(fullfile(params.cutouts.dir, '**/cutout*.jpg'));
nFiles = size(files,1);
cutout_imgnames_all = cell(1,nFiles);

for i=1:nFiles
    relativePath = extractAfter(files(i).folder, size(params.dataset.db.cutouts.dir,2)+1);
    cutout_imgnames_all{1,i} = fullfile(relativePath, files(i).name);
end

save(params.input.dblist.path);