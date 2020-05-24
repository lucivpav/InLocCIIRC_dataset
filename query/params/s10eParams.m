function [ params ] = s10eParams(params)

    params.query.dir = fullfile(params.dataset.dir, 'query-s10e');
    params.camera.rotation.wrt.marker = [0.0 0.0 1.5];
    params.camera.sensor.size = [3024, 4032]; % height, width
    params.camera.fl = 3172.435; % in pixels
    params.camera.origin.wrt.marker = 0.01 * [-3; 1; -4];
    
end