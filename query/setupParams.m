function [ params ] = setupParams(mode)
    % mode is one of {'s10e', 'annaHoloLens', 'pavelHoloLens'}

addpath('./params');
addpath('../functions/InLocCIIRC_utils/environment');

params = struct();
env = environment();

if strcmp(env, 'laptop')
    params.dataset.dir = '/Volumes/GoogleDrive/Můj disk/ARTwin/InLocCIIRC_dataset';
elseif strcmp(env, 'cmp')
    params.dataset.dir = '/mnt/datagrid/personal/lucivpav/InLocCIIRC_dataset';
end

if strcmp(mode, 's10eParams')
    params = s10eParams(params);
elseif strcmp(mode, 'holoLens1Params')
    params = holoLens1Params(params);
elseif strcmp(mode, 'holoLens2Params')
    params = holoLens2Params(params);
else
    error('Unrecognized mode');
end

params.mode = mode;
params.spaceName = 'B-315';
params.pointCloud.path = fullfile(params.dataset.dir, 'models', params.spaceName, 'cloud - rotated.ply');
params.projectPointCloudPy.path = '../functions/local/projectPointCloud/projectPointCloud.py';
params.reconstructPosePy.path = '../functions/local/reconstructPose/reconstructPose.py';
params.projectedPointCloud.dir = fullfile(params.query.dir, 'projectedPointCloud');
params.poses.dir = fullfile(params.query.dir, 'poses');
params.queryDescriptions.path = fullfile(params.query.dir, 'descriptions.csv');
params.rawPoses.path = fullfile(params.query.dir, 'rawPoses.csv');
params.cutouts.dir = fullfile(params.dataset.dir, 'cutouts');
params.inMap.tDiffMax = 1.3;
params.inMap.rotDistMax = 10; % in degrees
params.renderClosestCutouts = false;
params.closest.cutout.dir = fullfile(params.query.dir, 'closestCutout');

params.K = eye(3);
params.K(1,1) = params.camera.fl;
params.K(2,2) = params.camera.fl;
params.K(1,3) = params.camera.sensor.size(2)/2;
params.K(2,3) = params.camera.sensor.size(1)/2;

end
