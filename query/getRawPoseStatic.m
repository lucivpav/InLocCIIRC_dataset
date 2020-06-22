function [rawPosition, rawRotation] = getRawPoseStatic(queryIdx, queries, rawPosesTable)
    queryName = queries(queryIdx);
    queryNameWithoutExtension = strsplit(queryName, '.');
    queryNameWithoutExtension = queryNameWithoutExtension{1};
    rowIdx = find(rawPosesTable.id == str2double(queryNameWithoutExtension));

    rawPosition = [rawPosesTable{rowIdx,'x'}; rawPosesTable{rowIdx,'y'}; rawPosesTable{rowIdx,'z'}];
    rawRotation = [rawPosesTable{rowIdx,'alpha'}, rawPosesTable{rowIdx,'beta'}, rawPosesTable{rowIdx,'gamma'}];
end