function [rawPosition, rawRotation] = buildRawPose(queryIdx, queries, queryTable, measurementTable, syncConstant)
    queryName = queries(queryIdx);
    queryTimestamp = queryTable(find(strcmp(queryTable.name,queryName)), 'timestampMs');
    queryTimestamp = queryTimestamp{1,1};
    viconTimestamp = syncConstant + queryTimestamp; 
    [~, idx] = closest_value(measurementTable.timestampMs, double(viconTimestamp));
    closestEvent = measurementTable(idx,:);
    rawPosition = [closestEvent{1,'x'}; closestEvent{1,'y'}; closestEvent{1,'z'}];
    rawRotation = [closestEvent{1,'alpha'}, closestEvent{1,'beta'}, closestEvent{1,'gamma'}];
end