function [ params ] = s10eParams(params)
    % queries: [00132321090555753820.jpg, 00132321090868821963.jpg, 00132321091341754646.jpg, 00132321091488297196.jpg] aka 1,2,3,4

    params.query.dir = fullfile(params.dataset.dir, 'query-HoloLens1');
    %params.camera.rotation.wrt.marker = deg2rad([-6.0 6.0 -4.0]); % this is optimal for query 1
    %params.camera.rotation.wrt.marker = deg2rad([-5.0 6.0 -5.0]); % this is optimal for query 2
    %params.camera.rotation.wrt.marker = deg2rad([-8.0 6.0 -3.0]); % this is optimal for query 3
    %params.camera.rotation.wrt.marker = deg2rad([-8.0 6.0 -4.0]); % this is optimal for query 4
    params.camera.rotation.wrt.marker = deg2rad([-8.0 6.0 -4.0]); % this aims to be generic
    params.camera.origin.wrt.marker = 0.023 * [3; 20; -3];
    params.camera.sensor.size = [756, 1344]; % height, width
    params.camera.fl = 1015; % in pixels
    
end