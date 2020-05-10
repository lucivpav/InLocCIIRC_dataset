function [ params ] = holoLens2Params(params)
    % queries: [00132321103461934087.jpg, 00132321104757170200.jpg , 00132321103645112241.jpg] aka 1, 2, 3

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
    %params.camera.rotation.wrt.marker = deg2rad([-1.0 3.0 4.0]); % this is near optimal for query 2
    params.camera.rotation.wrt.marker = deg2rad([2.0 5.0 5.0]); % this is near optimal for query 3
    %params.camera.rotation.wrt.marker = deg2rad([-1.0 2.0 3.0]); % this is aims to be generic
    params.camera.originConstant = 0.023;
    %params.camera.origin.wrt.marker = params.camera.originConstant * [-3; 13; -5]; % this is near optimal for query 1
    %params.camera.origin.wrt.marker = params.camera.originConstant * [3; 13; -7]; % this is near optimal for query 2
    params.camera.origin.wrt.marker = params.camera.originConstant * [3; 16; -3]; % this is near optimal for query 3
    %params.camera.origin.wrt.marker = params.camera.originConstant * [3; 13; -5]; % this aims to be generic
    params.camera.sensor.size = [756, 1344]; % height, width
    params.camera.fl = 1015; % in pixels
    %params.HoloLensViconSyncConstant = 29.0 * 1000; % [ms]; expected: <28.8, 29.4> % this is near optimal for query 1
    %params.HoloLensViconSyncConstant = 29.0 * 1000; % [ms]; expected: <28.8, 29.4> % this is near optimal for query 2
    params.HoloLensViconSyncConstant = 29.0 * 1000; % [ms]; expected: <28.8, 29.4> % this is near optimal for query 3
    %params.HoloLensViconSyncConstant = 29.0 * 1000; % [ms]; expected: <28.8, 29.4> % this aims to be generic
    
end