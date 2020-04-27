function [ params ] = s10eParams(params)

    params.query.dir = fullfile(params.dataset.dir, 'query-HoloLens1');
    %params.camera.rotation.wrt.marker = deg2rad([-7.0 6.0 -4.0]); % this is optimal for 00132321090555753820.jpg
    params.camera.rotation.wrt.marker = deg2rad([-2.0 7.0 -7.0]); % this is optimal for 00132321090868821963.jpg
    %params.camera.rotation.wrt.marker = deg2rad([1.0 9.0 4.0]); % this is optimal for 00132321091341754646.jpg
    %params.camera.origin.wrt.marker = 0.023 * [5; 20; -3]; % this is optimal for 00132321090555753820.jpg
    params.camera.origin.wrt.marker = 0.023 * [5; 20; -3]; % this is optimal for 00132321090868821963.jpg
    params.camera.sensor.size = [756, 1344]; % height, width
    params.camera.fl = 1015; % in pixels
    
end