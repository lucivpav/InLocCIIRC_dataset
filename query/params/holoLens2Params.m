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
    params.camera.rotation.wrt.marker = deg2rad([4.0 0.0 5.0]); % this aims to be generic
    params.camera.origin.wrt.marker = 0.023 * [0; 20; -10];
    params.camera.sensor.size = [756, 1344]; % height, width
    params.camera.fl = 1015; % in pixels
    params.HoloLensViconSyncConstant = 29.1 * 1000; % [ms]; prev: <29.0, 29.3>
    
end