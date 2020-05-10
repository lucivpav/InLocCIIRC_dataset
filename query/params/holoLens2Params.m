function [ params ] = holoLens2Params(params)

    params.query.dir = fullfile(params.dataset.dir, 'query-HoloLens2');
    params.input.dir = '/Volumes/GoogleDrive/Můj disk/ARTwin/personal/lucivpav/HoloLens sequences';
    params.measurement.path = fullfile(params.input.dir, 'measurement2.txt');
    params.input.recording.dir = fullfile(params.input.dir, 'HoloLensRecording__2020_04_23__10_15_23');
    params.input.query.dir = fullfile(params.input.recording.dir, 'pv');
    params.input.poses.path = fullfile(params.input.recording.dir, 'pv_locationData.csv');
    params.HoloLensPoses.dir = fullfile(params.query.dir, 'HoloLensPoses');
    params.HoloLensProjectedPointCloud.dir = fullfile(params.query.dir, 'HoloLensProjectedPointCloud');
    params.HoloLensPosesDelay = 0; % in frames, w.r.t. reference poses
    %params.camera.rotation.wrt.marker = deg2rad([2.0 2.0 3.0]); % this is near optimal for query 1
    %params.camera.rotation.wrt.marker = deg2rad([2.0 5.0 5.0]); % this is near optimal for query 2
    %params.camera.rotation.wrt.marker = deg2rad([-1.0 3.0 4.0]); % this is near optimal for query 3
    %params.camera.rotation.wrt.marker = deg2rad([2.0 5.0 5.0]); % this is optimal for query 1 & 2
    %params.camera.rotation.wrt.marker = deg2rad([-1.0 3.0 4.0]); % this is optimal for query 3 & 4
    params.camera.rotation.wrt.marker = deg2rad([2.0 5.0 5.0]); % this is optimal for query 1 & 2 & 4
    params.camera.originConstant = 0.023;
    %params.camera.origin.wrt.marker = params.camera.originConstant * [-3; 13; -5]; % this is near optimal for query 1
    %params.camera.origin.wrt.marker = params.camera.originConstant * [3; 16; -3]; % this is near optimal for query 2
    %params.camera.origin.wrt.marker = params.camera.originConstant * [3; 13; -7]; % this is near optimal for query 3
    %params.camera.origin.wrt.marker = params.camera.originConstant * [1; 15; -7]; % this is optimal for query 1 & 2
    %params.camera.origin.wrt.marker = params.camera.originConstant * [2; 11; -3]; % this is optimal for query 3 & 4
    params.camera.origin.wrt.marker = params.camera.originConstant * [1; 15; -7]; % this is optimal for query 1 & 2 & 4
    params.camera.sensor.size = [756, 1344]; % height, width
    params.camera.fl = 1015; % in pixels
    %params.HoloLensViconSyncConstant = 29.0 * 1000; % [ms]; expected: <28.8, 29.4> % this is near optimal for query 1
    %params.HoloLensViconSyncConstant = 29.0 * 1000; % [ms]; expected: <28.8, 29.4> % this is near optimal for query 2
    %params.HoloLensViconSyncConstant = 29.0 * 1000; % [ms]; expected: <28.8, 29.4> % this is near optimal for query 3
    %params.HoloLensViconSyncConstant = 29.1 * 1000; % [ms]; expected: <28.8, 29.4> % this is optimal for query 1 & 2
    %params.HoloLensViconSyncConstant = 29.2 * 1000; % [ms]; expected: <28.8, 29.4> % this is optimal for query 3 & 4
    params.HoloLensViconSyncConstant = 29.1 * 1000; % [ms]; expected: <28.8, 29.4> % this is optimal for query 1 & 2 & 4

    % NOTE: params for 1 & 2 & 4 are the same as for 1 & 2
    
end