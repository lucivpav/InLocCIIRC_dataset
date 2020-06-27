function [rawPosition, rawRotation] = getRawPose(queryIdx, queries, queryTable, measurementTable, rawPosesTable, params)
% either queryTable, measurementTable are set, or
% rawPosesTable is set
% those values that are not set, should be set to false
% TDOO: rename to sth like getViconPoseWrtMarker

if islogical(rawPosesTable) && rawPosesTable == false
    useRawPosesTable = false;
else
    useRawPosesTable = true;
end

if useRawPosesTable
    [rawPosition, rawRotation] = getRawPoseStatic(queryIdx, params.interestingQueries, rawPosesTable);
else
    [rawPosition, rawRotation] = getRawPoseSequential(queryIdx, params.interestingQueries, queryTable, ...
                                                        measurementTable, params.HoloLensViconSyncConstant);
end

end