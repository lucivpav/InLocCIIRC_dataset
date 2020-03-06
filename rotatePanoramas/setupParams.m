function [ params ] = setupParams

params = struct();

params.dataset.dir = '/Volumes/GoogleDrive/Mùj disk/ARTwin/InLocCIIRC_dataset';
params.spaceName = 'B-670';
params.sweepData.json.path = fullfile(params.dataset.dir, 'sweepData', sprintf('%s.json', params.spaceName));
params.sweepData.mat.path = fullfile(params.dataset.dir, 'sweepData', sprintf('%s.mat', params.spaceName));
params.pointCloud.path = fullfile(params.dataset.dir, 'models', params.spaceName, 'cloud - rotated.ply');
params.panoramas.dir = fullfile(params.dataset.dir, 'panoramas', params.spaceName);
params.panorama2pointClouds.dir = fullfile(params.dataset.dir, 'panoramas2pointClouds', params.spaceName);
params.rotatedPanoramas.dir = fullfile(params.dataset.dir, 'rotatedPanoramas', params.spaceName);
params.temporary.dir = fullfile(params.dataset.dir, 'temp');
params.projectPointCloudPy.path = '../projectPointCloud/projectPointCloud.py';

end