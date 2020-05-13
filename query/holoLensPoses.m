% 1. Convert the HoloLens poses to poses w.r.t. the model
% 2. Evaluate the transfmored poses w.r.t. reference poses (Vicon raw poses transformed)

addpath('../functions/local/projectPointCloud');
addpath('../functions/InLocCIIRC_utils/rotationDistance');
addpath('../functions/InLocCIIRC_utils/mkdirIfNonExistent');
addpath('../functions/InLocCIIRC_utils/load_CIIRC_transformation');
addpath('../functions/InLocCIIRC_utils/P_to_str');
addpath('../functions/local/R_to_numpy_array');
addpath('../functions/InLocCIIRC_utils/rotationMatrix');
[ params ] = setupParams('holoLens1Params'); % NOTE: tweak

projectPC = false; % NOTE: tweak

%% build HoloLens poses table w.r.t. to HoloLens CS
descriptionsTable = readtable(params.queryDescriptions.path); % decribes the reference poses
rawHoloLensPosesTable = readtable(params.input.poses.path);
assert(size(descriptionsTable,1) == size(rawHoloLensPosesTable,1));
nQueries = size(descriptionsTable,1);

blacklistedQueries = false(1,nQueries);
blacklistedQueries(params.blacklistedQueryInd) = true;
nBlacklistedQueries = sum(blacklistedQueries);
fprintf('You have blacklisted %0.0f%% queries. %d queries remain.\n', ...
            nBlacklistedQueries*100/nQueries, nQueries-nBlacklistedQueries);
whitelistedQueries = logical(ones(1,nQueries) - blacklistedQueries); % w.r.t. reference frames

blacklistedQueryIndHL = params.blacklistedQueryInd + params.HoloLensPosesDelay;
blacklistedQueryIndHL = blacklistedQueryIndHL(blacklistedQueryIndHL < nQueries);
blacklistedQueriesHL = false(1,nQueries);
blacklistedQueriesHL(blacklistedQueryIndHL) = true;
whitelistedQueriesHL = logical(ones(1,nQueries) - blacklistedQueriesHL); % w.r.t. HoloLens frames

nPts = sum(whitelistedQueries);
%nPts = sum(whitelistedQueries)*4;
pts = zeros(nPts,3);
idx = 1;
for i=1:nQueries
    id = descriptionsTable{i, 'id'};
    %space = descriptionsTable{i, 'space'}{1,1};
    %inMap = descriptionsTable{i, 'inMap'};
    t = [rawHoloLensPosesTable{i, 'Position_X'}; ...
                rawHoloLensPosesTable{i, 'Position_Y'}; ...
                rawHoloLensPosesTable{i, 'Position_Z'}];
    orientation = [rawHoloLensPosesTable{i, 'Orientation_W'}, ...
                    rawHoloLensPosesTable{i, 'Orientation_X'}, ...
                    rawHoloLensPosesTable{i, 'Orientation_Y'}, ...
                    rawHoloLensPosesTable{i, 'Orientation_Z'}];
    R = rotmat(quaternion(orientation), 'frame');
    
    % camera points to -z in HoloLens
    % see https://docs.microsoft.com/en-us/windows/mixed-reality/coordinate-systems-in-directx
    rFix = rotationMatrix([pi, 0.0, 0.0], 'ZYX');
    R = rFix * R;
    
    P = eye(4);
    P(1:3,1:3) = R;
    P(1:3,4) = R * -t;
    Ps{i} = P;
    
    if ~whitelistedQueriesHL(i)
        continue;
    end
    
    pts(idx,:) = t';
%     pts((idx-1)*4+1,:) = t';
%     pts((idx-1)*4+2,:) = t' + R(1,:);
%     pts((idx-1)*4+3,:) = t' + R(2,:);
%     pts((idx-1)*4+4,:) = t' + R(3,:);
    idx = idx + 1;
end
Ps = {Ps};
Ps = Ps{1,1};
Ps = reshape(Ps, nQueries, 1);
ids = {descriptionsTable.id};
ids = ids{1,1};
ids = reshape(ids, nQueries, 1);
holoLensPosesTable = table(ids, Ps);
holoLensPosesTable.Properties.VariableNames = {'id', 'P'};

%% extract reference poses
pts_ref = zeros(nPts,3);
idx = 1;
for i=1:nQueries
    id = holoLensPosesTable{i, 'id'};
    P_ref = load_CIIRC_transformation(fullfile(params.poses.dir, sprintf('%d.txt', id)));
    R_ref = P_ref(1:3,1:3);
    T_ref = -inv(R_ref)*P_ref(1:3,4);
    
    if ~whitelistedQueries(i)
        continue;
    end
    
    pts_ref(idx,:) = T_ref';
%     pts_ref((idx-1)*4+1,:) = T_ref';
%     pts_ref((idx-1)*4+2,:) = T_ref' + R_ref(1,:);
%     pts_ref((idx-1)*4+3,:) = T_ref' + R_ref(2,:);
%     pts_ref((idx-1)*4+4,:) = T_ref' + R_ref(3,:);
    idx = idx + 1;
end

%% build HoloLens poses table w.r.t. to model CS

% due to a (possible) delay, we need to match frames between HoloLens and reference (Vicon)
pts = pts(params.HoloLensPosesDelay+1:size(pts,1),:);
pts_ref = pts_ref(1:size(pts_ref,1)-params.HoloLensPosesDelay,:);

A = eye(4);
[d,Z,transform] = procrustes(pts_ref, pts, 'scaling', false, 'reflection', false);
R = transform.T;
A(1:3,1:3) = R; % NOTE: first, R must be correct, then t can be correct
A(1:3,4) = -R*transform.c(1,:)';

for i=1:nQueries
    P = holoLensPosesTable.P{i};
    P = P * A; % why is this not A * P ??
    holoLensPosesTable.P{i} = P;
    t = -inv(P(1:3,1:3))*P(1:3,4);
    Ts{i} = t;
end
Ts = reshape(Ts, nQueries, 1);

%% store the HoloLens poses for future reference
mkdirIfNonExistent(params.HoloLensPoses.dir);
readmeFile = fopen(fullfile(params.HoloLensPoses.dir, 'readme.txt'), 'w');
fprintf(readmeFile, 'HoloLens poses w.r.t model (Matterport) CS.\nErrors are w.r.t. reference poses.');
fclose(readmeFile);
for i=1:nQueries
    id = holoLensPosesTable{i, 'id'};
    P = holoLensPosesTable.P{i};
    poseFile = fopen(fullfile(params.HoloLensPoses.dir, sprintf('%d.txt', id)), 'w');
    P_str = P_to_str(P);
    fprintf(poseFile, '%s', P_str);
    fclose(poseFile);
end

%% Evaluate w.r.t reference poses
% NOTE: these errors are w.r.t. HoloLens frames. This means that for
% params.HoloLensPosesDelay > 1, the first pose in HL frames does not a
% reference pose. Also, the last pose in reference frames does not have a
% matching HoloLens pose.
clearvars errors
errors(nQueries,1) = struct();
for i=1:nQueries % TODO: ugly init, due to my lack of MATLAB understanding
    id = holoLensPosesTable{i, 'id'};
    errors(i).queryId = id;
    errors(i).translation = -1; % this means we couldn't verify the,
    errors(i).orientation = -1; % correctness due to the delay
end

%%
for i=1:nQueries
    id = holoLensPosesTable{i, 'id'};
    id = id - params.HoloLensPosesDelay;
    if id < 1
        continue;
    end
    P = holoLensPosesTable.P{i};
    referencePosePath = fullfile(params.poses.dir, sprintf('%d.txt', id));
    P_ref = load_CIIRC_transformation(referencePosePath);
    T = -inv(P(1:3,1:3))*P(1:3,4);
    T_ref = -inv(P_ref(1:3,1:3))*P_ref(1:3,4);
    R = P(1:3,1:3);
    R_ref = P_ref(1:3,1:3);
    errors(i).translation = norm(T - T_ref);
    errors(i).orientation = rotationDistance(R_ref, R);
end

relevancyArray = logical(([errors.translation] ~= -1) .* whitelistedQueriesHL);
relevantErrors = errors(relevancyArray);
avgTerror = mean(cell2mat({relevantErrors.translation}));
avgRerror = mean(cell2mat({relevantErrors.orientation}));
summaryMessage = sprintf('Mean errors (whitelist only): translation: %0.2f [m], orientation: %0.f [deg]\n', avgTerror, avgRerror);
fprintf(summaryMessage);
summaryFile = fopen(fullfile(params.HoloLensPoses.dir, 'errorSummary.txt'), 'w');
fprintf(summaryFile, summaryMessage);
fclose(summaryFile);

%% write errors to file
for i=1:nQueries
    translation2{i} = sprintf('%0.2f', errors(i).translation);
    orientation2{i} = sprintf('%0.2f', errors(i).orientation);
end
errorsTable = table({errors.queryId}', translation2', orientation2');
errorsTable.Properties.VariableNames = {'id', 'translation', 'orientation'};
errorsPath = fullfile(params.HoloLensPoses.dir, 'errors.csv');
writetable(errorsTable, errorsPath);

%% visualize error distributions (whitelist only)
tiledlayout(2,1);

nexttile
histogram(cell2mat({relevantErrors.translation}));
title('HoloLens to Matterport poses: Translation errors (whitelist only)');
xlabel('Translation error [m]');
ylabel('Number of occurences');

nexttile
histogram(cell2mat({relevantErrors.orientation}));
title('HoloLens to Matterport poses: Orientation errors (whitelist only)');
xlabel('Orientation error [deg]');
ylabel('Number of occurences');

%% project PC using the transformed HoloLens poses
if ~projectPC
    return;
end
figure();
mkdirIfNonExistent(params.HoloLensProjectedPointCloud.dir);
for i=1:nQueries
    id = holoLensPosesTable{i, 'id'};
    P = holoLensPosesTable.P{i};
    R = P(1:3,1:3);
    t = -inv(R)*P(1:3,4);
    
    pointSize = 8.0;
    f = params.camera.fl; % in pixels
    sensorSize = params.camera.sensor.size; % height, width
    outputSize = sensorSize;
    projectedPointCloud = projectPointCloud(params.pointCloud.path, f, R, ...
                                        t, sensorSize, outputSize, pointSize, ...
                                        params.projectPointCloudPy.path);
                                    
    imshow(projectedPointCloud);
    outPCFilename = sprintf('%d-PC.jpg', id);
    outPCPath = fullfile(params.HoloLensProjectedPointCloud.dir, outPCFilename);
    imwrite(projectedPointCloud, outPCPath);

    idRef = id - params.HoloLensPosesDelay;
    if idRef < 1
        continue;
    end

    queryFilename = sprintf('%d.jpg', idRef);
    queryImg = imread(fullfile(params.query.dir, queryFilename));
    outQueryFilename = sprintf('%d-query-original-%d.jpg', id, idRef);
    outQueryPath = fullfile(params.HoloLensProjectedPointCloud.dir, outQueryFilename);
    imwrite(queryImg, outQueryPath);
end