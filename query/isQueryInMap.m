function [inMap, closestCutout] = isQueryInMap(queryT, queryDir, querySpace, cutoutDescriptions, params)
    closestCutout.tDiff = inf;
    closestCutout.dirDiff = inf;
    inMap = false;
    for i=1:size(cutoutDescriptions,2)
        cutout = cutoutDescriptions(i);
        if ~strcmp(querySpace, cutout.space)
            continue
        end
        tDiff = norm(queryT - cutout.position);
        dirDiff = atan2d(norm(cross(queryDir,cutout.direction)),dot(queryDir,cutout.direction));
        if tDiff < closestCutout.tDiff || (tDiff == closestCutout.tDiff && dirDiff < closestCutout.dirDiff)
            closestCutout.tDiff = tDiff;
            closestCutout.dirDiff = dirDiff;
            closestCutoutIdx = i;
            closestCutout.name = cutout.name;
            closestCutout.space = cutout.space;
        end
        if tDiff < params.inMap.tDiffMax && dirDiff < params.inMap.dirDiffMax
            inMap = true;
        end
    end
    
    desc = cutoutDescriptions(closestCutoutIdx);
    cutoutName = desc.name;
    parsed = strsplit(cutoutName, '_');
    sweepId = parsed(2);
    cutoutImagePath = fullfile(params.cutouts.dir, desc.space, sweepId, cutoutName);
    cutoutImagePath = cutoutImagePath{1};
    closestCutout.img = imread(cutoutImagePath);
end