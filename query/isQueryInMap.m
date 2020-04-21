function [inMap, closestCutout] = isQueryInMap(P, querySpace, cutoutDescriptions, params)
    addpath('../functions/InLocCIIRC_utils/rotationDistance');
    closestCutout.tDiff = inf;
    closestCutout.rotDist = inf;
    inMap = false;
    for i=1:size(cutoutDescriptions,2)
        cutout = cutoutDescriptions(i);
        if ~strcmp(querySpace, cutout.space)
            continue
        end
        queryT = -inv(P(1:3,1:3))*P(1:3,4);
        tDiff = norm(queryT - cutout.position);
        rFix = rotationMatrix([pi, 0.0, 0.0], 'ZYX'); % cutout.R is pointing to -z
        cutoutR = cutout.R * rFix;
        rotDist = rotationDistance(P(1:3,1:3), cutoutR);
        if tDiff < closestCutout.tDiff || (tDiff == closestCutout.tDiff && rotDist < closestCutout.rotDist)
            closestCutout.tDiff = tDiff;
            closestCutout.rotDist = rotDist;
            closestCutoutIdx = i;
            closestCutout.name = cutout.name;
            closestCutout.space = cutout.space;
        end
        if tDiff < params.inMap.tDiffMax && rotDist < params.inMap.rotDistMax
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