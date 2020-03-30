function desc = buildCutoutDescriptions(params)
    cutouts = dir(fullfile(params.cutouts.dir, '**/cutout*.jpg'));
    nCutouts = size(cutouts,1);
    cutouts = struct2cell(cutouts);
    cutoutNames = cutouts(1,:);
    cutoutSpaces = cutouts(2,:);
    for i=1:nCutouts
        cutoutSpace = cutoutSpaces(i);
        cutoutSpace = cutoutSpace{1};
        cutoutSpace = strsplit(cutoutSpace, '/');
        cutoutSpace = cutoutSpace(end-1);
        cutoutSpace = cutoutSpace{1};
        desc(i).space = cutoutSpace;
    end
    
    for i=1:size(cutoutNames,2)
        cutoutName = cutoutNames(i);
        cutoutName = cutoutName{1};
        desc(i).name = cutoutName;
        parsed = strsplit(cutoutName, '_');
        sweepId = parsed(2);        
        sweepId = str2num(sweepId{1});
        yaw = parsed(3);
        yaw = str2double(yaw{1});
        pitch = parsed(4);
        pitch = strsplit(pitch{1}, '.');
        pitch = pitch(1);
        pitch = str2double(pitch{1});
        sweepDataPath = fullfile(params.dataset.dir, 'sweepData', [desc(i).space, '.mat']);
        load(sweepDataPath, 'sweepData');
        fun = @(x) sweepData(x).panoId == sweepId;
        tf = arrayfun(fun, 1:numel(sweepData));
        sweepRecord = sweepData(tf);
        
        cameraRotation = sweepRecord.rotation + [pitch, -yaw, 0.0];
        R = rotationMatrix(deg2rad(cameraRotation), 'ZYX');
        desc(i).direction = R * [0.0; 0.0; -1.0];
        desc(i).position = sweepRecord.position;
    end
end