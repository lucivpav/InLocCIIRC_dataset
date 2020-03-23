function [ params ] = setupParams

params = struct();

params.dataset.dir = '/Volumes/GoogleDrive/Můj disk/ARTwin/InLocCIIRC_dataset';
params.spaceName = 'B-315';
params.pointCloud.path = fullfile(params.dataset.dir, 'models', params.spaceName, 'cloud - rotated.ply');
params.projectPointCloudPy.path = '../projectPointCloud/projectPointCloud.py';
params.query.dir = fullfile(params.dataset.dir, 'query');
params.projectedPointCloud.dir = fullfile(params.query.dir, 'projectedPoiontCloud');
params.poses.path = fullfile(params.query.dir, 'poses.csv');
params.rawPoses.path = fullfile(params.query.dir, 'rawPoses.csv');

end