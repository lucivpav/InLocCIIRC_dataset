addpath('../functions/local/reconstructPose');
addpath('../functions/local/R_to_numpy_array');

close all

[ params ] = setupParams('s10eParams');

%% find ground truth camera poses
queryInd = 1:size(params.interestingQueries,2);
%queryInd = [1];
nInterestingQueries = size(queryInd,2);

% TODO: which result from P3P should be considered??? The Rs and Cs seem pretty inconsistent!
j=1;
%for j=1:50
%fprintf('Picking up the %d. lowest error pose from P3P.\n', j);

for i=1:nInterestingQueries
    queryIdx = queryInd(i);
    [Rs,Cs,errors] = reconstructPose(params.interestingPointsQuery{queryIdx}, params.interestingPointsPC{queryIdx}, ...
                            params.K, params.reconstructPosePy.path);
    [lowestErrors,lowestErrorInd] = sort(errors);

    chosenIdx = lowestErrorInd(j);
    cameraRotations{i} = reshape(Rs(chosenIdx,:,:), 3,3); % aka modelToCamera(1:3,1:3)
    cameraPositions{i} = reshape(Cs(chosenIdx,:), 3,1); % wrt model
end

if strcmp(params.mode, 's10eParams')
    measurementTable = false;
    queryTable = false;
    rawPosesTable = readtable(params.rawPoses.path);
else
    [measurementTable, queryTable, ~] = initiMeasurementAndQueryTables(params);
    rawPosesTable = false;
    tDiffMs = 0.0; % try different sync constants?
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
    markerRotations{i} = R; % columns are model bases wrt marker
    markerPositions{i} = t; % wrt model
end

%% find the individual (optimal) params
optimalParams = params;
for i=1:nInterestingQueries
    queryIdx = queryInd(i);

    markerRotation = markerRotations{i};
    cameraRotation = cameraRotations{i};

    optimalTranslationsNonRelative{i} = markerRotation * (cameraPositions{i} - markerPositions{i});
    optimalTranslations{i} = optimalTranslationsNonRelative{i} / params.camera.originConstant;

    % optimalRs{i} aka cameraToMarker(1:3,1:3), i.e. columns are bases of camera (epsilon) in marker
    % cameraRotation aka modelToCamera(1:3,1:3)
    % markerRotation aka modelToMarker(1:3,1:3)
    % inv(modelToMarker) * cameraToMarker = inv(modelToCamera)
    % cameraToMarker = modelToMarker * inv(modelToCamera)
    optimalRs{i} = markerRotation * inv(cameraRotation);
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

    [projectionErrorSum, ~,~] = evaluateMatches([queryIdx], testParams, queryTable, measurementTable, rawPosesTable);
end
optimalParams.optimal.camera.origin.relative.wrt.marker = containers.Map(queryInd, optimalTranslations);
optimalParams.optimal.camera.rotation.wrt.marker = containers.Map(queryInd, optimalRotations);

%% suggest a generic transformation
optimalGenericOrigin = mean(cell2mat(optimalTranslations)', 1);
optimalGenericRotation = mean(cell2mat(optimalRotations'), 1);

%optimalGenericOrigins{j,1} = optimalGenericOrigin;
%optimalGenericRotations{j,1} = optimalGenericRotation;
%projectionErrorSums{j,1} = projectionErrorSum;

%end
%return

optimalParams.camera.origin.relative.wrt.marker = optimalGenericOrigin';
optimalParams.camera.origin.wrt.marker = params.camera.originConstant * optimalGenericOrigin';
optimalParams.camera.rotation.wrt.marker = optimalGenericRotation;
%for i=1:nInterestingQueries fprintf('[%0.4f, %0.4f, %0.4f],\n', optimalRotations{i}); end
%for i=1:nInterestingQueries fprintf('[%0.4f; %0.4f; %0.4f],\n', optimalTranslations{i}); end

%% Display mean, standard deviation. Evaluate all interesting queries on the suggested transformation
fprintf('Mean of optimal origins: %0.4f %0.4f %0.4f\n', optimalGenericOrigin);
fprintf('Mean of optimal rotations: %0.4f %0.4f %0.4f\n', optimalGenericRotation);
fprintf('\n');
sdOrigins = std(cell2mat(optimalTranslations)', 0, 1);
sdRotations = std(cell2mat(optimalRotations'), 0, 1);
fprintf('Standard deviation of optimal origins: %0.4f %0.4f %0.4f\n', sdOrigins);
fprintf('Standard deviation of optimal rotations: %0.4f %0.4f %0.4f\n', sdRotations);
fprintf('\n');

fprintf('Evaluation of the suggested generic transformation:\n');
evaluateMatches(queryInd, optimalParams, queryTable, measurementTable, rawPosesTable);