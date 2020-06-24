params = struct();
params.dataset.dir = '/Volumes/GoogleDrive/M�j disk/ARTwin/InLocCIIRC_dataset_mirror';
params.spaceName = 'B-315';
%params.sweepData.mat.path = fullfile(params.dataset.dir, 'sweepData', sprintf('%s.mat', params.spaceName));
params.model.path = fullfile(params.dataset.dir, 'models', params.spaceName, 'cloud - rotated.ply');
params.output.path = fullfile(params.dataset.dir, 'scans', sprintf('%s.ptx.mat', params.spaceName));

%%
pc = pcread(params.model.path);
x = pc.Location(:,1);
y = pc.Location(:,2);
z = pc.Location(:,3);
intensity = ones(size(x,1), 1);
r = pc.Color(:,1);
g = pc.Color(:,2);
b = pc.Color(:,3);

A = {x, y, z, intensity, r, g, b};

% TODO: nCol, nRow, S, T
save(params.output.path, 'A');