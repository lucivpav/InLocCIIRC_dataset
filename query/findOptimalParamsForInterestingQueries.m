addpath('../functions/local/reconstructPose');
addpath('../functions/local/R_to_numpy_array');

close all

[ params ] = setupParams('s10eParams');

%% find ground truth camera poses

% try different sync constants?
tDiffMs = 0.0;

queryInd = 1:size(params.interestingQueries,2);
nInterestingQueries = size(queryInd,2);
for i=1:nInterestingQueries
    queryIdx = queryInd(i);
    [R,C] = reconstructPose(params.interestingPointsQuery{queryIdx}, params.interestingPointsPC{queryIdx}, ...
                            params.K, params.reconstructPosePy.path);
    cameraRotations{i} = R; % wrt model
    cameraPositions{i} = C; % wrt model
end

if strcmp(params.mode, 's10eParams')
    measurementTable = false;
    queryTable = false;
    rawPosesTable = readtable(params.rawPoses.path);
else
    [measurementTable, queryTable, ~] = initiMeasurementAndQueryTables(params);
    rawPosesTable = false;
    params.HoloLensViconSyncConstant = double(params.HoloLensViconSyncConstant + tDiffMs);
end

%% check p3p actually works
%evaluateMatchesUsingPs(queryInd, params, cameraRotations, cameraPositions);
%return

%% find marker poses
for i=1:nInterestingQueries
    queryIdx = queryInd(i);

    [rawPosition, rawRotation] = getRawPose(queryIdx, params.interestingQueries, queryTable, ...
                                                measurementTable, rawPosesTable, params);

    %% project and check whether it corresponds to the initial sequence image
    paramsCopy = params;
    paramsCopy.camera.origin.relative.wrt.marker = [0; 0; 0];
    paramsCopy.camera.origin.wrt.marker = [0; 0; 0];
    paramsCopy.camera.rotation.wrt.marker = [0.0 0.0 0.0];
    [R, t] = rawPoseToPose(rawPosition, rawRotation, paramsCopy);
    markerRotations{i} = R; % wrt model
    markerPositions{i} = t; % wrt model
end

%% find the individual (optimal) params
optimalParams = params;
for i=1:nInterestingQueries
    queryIdx = queryInd(i);

    % bring y to where z is, (undo the format required by projection)
    rFix = rotationMatrix([pi/2, 0.0, 0.0], 'ZYX');
    markerRotation = markerRotations{i} * rFix;
    cameraRotation = cameraRotations{i} * rFix;

    optimalTranslationsNonRelative{i} = inv(markerRotation) * (cameraPositions{i} - markerPositions{i});
    optimalTranslations{i} = optimalTranslationsNonRelative{i} / params.camera.originConstant;

    optimalRs{i} = inv(markerRotation) * cameraRotation;
    optimalRotations{i} = rad2deg(rotm2eul(optimalRs{i}, 'XYZ')); % TODO: WTF, shouldn't this be ZYX?!
        % anyways, I have experimentally shown that I can indeed recover optimalRs{i} from optimalRotations{i} by
        % rotationMatrix(deg2rad(optimalRotations{i}), 'XYZ'), which is done in rawPoseToPose;

    testParams = optimalParams;
    testParams.camera.origin.relative.wrt.marker = optimalTranslations{i};
    testParams.camera.origin.wrt.marker = optimalTranslationsNonRelative{i};
    testParams.camera.rotation.wrt.marker = optimalRotations{i};

    % brute-force step sensitivity test
    % NOTE: the noise values should be half of the step size
    %testParams.camera.origin.relative.wrt.marker = testParams.camera.origin.relative.wrt.marker + 0.5;
    %testParams.camera.origin.wrt.marker = params.camera.originConstant * testParams.camera.origin.relative.wrt.marker;
    %testParams.camera.rotation.wrt.marker = testParams.camera.rotation.wrt.marker + 0.25;

    evaluateMatches([queryIdx], testParams, queryTable, measurementTable, rawPosesTable);
end
optimalParams.optimal.camera.origin.relative.wrt.marker = containers.Map(queryInd, optimalTranslations);
optimalParams.optimal.camera.rotation.wrt.marker = containers.Map(queryInd, optimalRotations);

%% suggest a generic transformation
optimalGenericOrigin = mean(cell2mat(optimalTranslations)');
optimalGenericRotation = mean(cell2mat(optimalRotations'));

optimalParams.camera.origin.relative.wrt.marker = optimalGenericOrigin';
optimalParams.camera.origin.wrt.marker = optimalGenericOrigin' / params.camera.originConstant;
optimalParams.camera.rotation.wrt.marker = optimalGenericRotation;