function [ params ] = s10eParams(params)

    params.query.dir = fullfile(params.dataset.dir, 'query-HoloLens1');
    params.camera.rotation.wrt.marker = deg2rad([5.0 0.0 0.0]); % TODO
    params.camera.sensor.size = [756, 1344]; % height, width
    params.camera.fl = 1015; % in pixels
    params.camera.origin.wrt.marker = 0.01 * [3; 10; -9]; % TODO
    
end