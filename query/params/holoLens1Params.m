function [ params ] = holoLens1Params(params)
    % queries: [00132321090555753820.jpg, 00132321090868821963.jpg, 00132321091341754646.jpg, 00132321091488297196.jpg] aka 1,2,3,4

    params.query.dir = fullfile(params.dataset.dir, 'query-HoloLens1');
    params.input.dir = '/Volumes/GoogleDrive/Můj disk/ARTwin/personal/lucivpav/HoloLens sequences';
    params.measurement.path = fullfile(params.input.dir, 'measurement1.txt');
    params.input.recording.dir = fullfile(params.input.dir, 'HoloLensRecording__2020_04_23__09_53_01');
    params.input.query.dir = fullfile(params.input.recording.dir, 'pv');
    params.input.poses.path = fullfile(params.input.recording.dir, 'pv_locationData.csv');
    params.HoloLensPoses.dir = fullfile(params.query.dir, 'HoloLensPoses');
    params.HoloLensProjectedPointCloud.dir = fullfile(params.query.dir, 'HoloLensProjectedPointCloud');
    params.HoloLensTranslationDelay = 4; % in frames, w.r.t. reference poses
    params.HoloLensOrientationDelay = 4; % in frames, w.r.t. reference poses

    % NOTE: some reference poses are wrong due to Vicon error, blacklist them
    params.blacklistedQueryInd = [103:109, 162, 179:188, 191:193, 286:288];

    %params.camera.rotation.wrt.marker = deg2rad([-6.0 6.0 -4.0]); % this is optimal for query 1
    %params.camera.rotation.wrt.marker = deg2rad([-5.0 6.0 -5.0]); % this is optimal for query 2
    %params.camera.rotation.wrt.marker = deg2rad([-8.0 6.0 -3.0]); % this is optimal for query 3
    %params.camera.rotation.wrt.marker = deg2rad([-8.0 6.0 -4.0]); % this is optimal for query 4
    params.camera.rotation.wrt.marker = deg2rad([-8.0 6.0 -4.0]); % this aims to be generic
    params.camera.origin.wrt.marker = 0.023 * [3; 20; -3];
    params.camera.sensor.size = [756, 1344]; % height, width
    params.camera.fl = 1015; % in pixels
    params.HoloLensViconSyncConstant = 10.7 * 1000; % [ms]
    
end