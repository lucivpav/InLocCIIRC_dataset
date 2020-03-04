function [ params ] = setupParams

params = struct();

params.dataset.dir = '/Volumes/GoogleDrive/Mùj disk/ARTwin/InLocCIIRC_dataset';
params.spaceName = 'B-315';
params.pointCloud.path = fullfile(params.dataset.dir, 'models', params.spaceName, 'cloud - rotated.ply');
params.temporary.dir = fullfile(params.dataset.dir, 'temp');
params.projectPointCloudPy.path = '../projectPointCloud/projectPointCloud.py';

end