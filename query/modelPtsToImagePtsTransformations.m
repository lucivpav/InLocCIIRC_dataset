% this code stores 3D interesting points wrt all intermediate coordinate systems
addpath('../functions/InLocCIIRC_utils/rotationDistance');
addpath('../functions/InLocCIIRC_utils/params');

[ params ] = setupParams('holoLens1');
queryInd = 1:size(params.interestingQueries,2);

if strcmp(params.mode, 's10eParams')
    measurementTable = false;
    queryTable = false;
    rawPosesTable = readtable(params.rawPoses.path);
else
    [measurementTable, queryTable, ~] = initiMeasurementAndQueryTables(params);
    rawPosesTable = false;
end

nQueries = size(queryInd,2);
for i=1:nQueries
    queryIdx = queryInd(i);
    [markerOriginWrtVicon, markerRotationWrtVicon] = getRawPose(queryIdx, params.interestingQueries, queryTable, ...
                                            measurementTable, rawPosesTable, params);
    [modelToVicon, viconToMarker, markerToCamera, cameraToImage] = getModelToImageTransformations(markerOriginWrtVicon, ...
                                                                                                    markerRotationWrtVicon, params);
    thisInterestingPoints = params.interestingPointsPC{queryIdx};
    nCorrespondences = size(thisInterestingPoints,2);
    thisTransData.modelToVicon = modelToVicon;
    thisTransData.viconToMarker = viconToMarker;
    thisTransData.markerToCamera = markerToCamera;
    thisTransData.cameraToImage = cameraToImage;
    thisTransData.queryName = params.interestingQueries(queryIdx);
    thisTransData.interestingPointsWrtModel = thisInterestingPoints;
    thisTransData.interestingPointsWrtVicon = modelToVicon * [thisInterestingPoints; ones(1,nCorrespondences)];
    thisTransData.interestingPointsWrtMarker = viconToMarker * thisTransData.interestingPointsWrtVicon;
    thisTransData.interestingPointsWrtCamera = markerToCamera * thisTransData.interestingPointsWrtMarker;
    thisTransData.interestingPointsWrtImage = cameraToImage * thisTransData.interestingPointsWrtCamera;
    thisTransData.interestingPointsWrtImage = thisTransData.interestingPointsWrtImage ./ thisTransData.interestingPointsWrtImage(3,:);
    thisTransData.interestingPointsWrtVicon = thisTransData.interestingPointsWrtVicon(1:3,:);
    thisTransData.interestingPointsWrtMarker = thisTransData.interestingPointsWrtMarker(1:3,:);
    thisTransData.interestingPointsWrtCamera = thisTransData.interestingPointsWrtCamera(1:3,:);
    thisTransData.interestingPointsWrtImage = thisTransData.interestingPointsWrtImage(1:2,:);
    thisTransData.referenceInterestingPointsProjection = params.interestingPointsQuery{queryIdx};
    transData(i) = thisTransData;

    %% verify the transformations are correct
    [R, t] = rawPoseToPose(markerOriginWrtVicon, markerRotationWrtVicon, params); 
    P = [params.camera.K*R, -params.camera.K*R*t];
    projectedPoints = projectPointsUsingP(thisInterestingPoints, P);
    discrepancies = vecnorm(projectedPoints - thisTransData.interestingPointsWrtImage, 2);
    assert(all(discrepancies < 1e-6));
end

filename = 'modelPtsToImagePtsTransformations-naive.mat';
%filename = 'modelPtsToImagePtsTransformations-optimized.mat';
camera = params.camera;
vicon = params.vicon;
HoloLensViconSyncConstant = params.HoloLensViconSyncConstant;
save(fullfile(params.dataset.query.dir, filename), 'transData', 'camera', 'HoloLensViconSyncConstant', 'vicon');