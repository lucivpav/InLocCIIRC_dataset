function [ params ] = s10eParams(params)

    params.query.dir = fullfile(params.dataset.dir, 'query-HoloLens1');
    %params.camera.rotation.wrt.marker = deg2rad([-10.0 3.0 -5.0]);
    params.camera.rotation.wrt.marker = deg2rad([-7.0 6.0 -4.0]);
    params.camera.sensor.size = [756, 1344]; % height, width
    params.camera.fl = 1015; % in pixels
    params.camera.origin.wrt.marker = 0.023 * [5; 20; -3];
    
end