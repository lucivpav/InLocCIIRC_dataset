% 1. Convert the HoloLens poses to poses w.r.t. the model
% 2. Evaluate the transfmored poses w.r.t. reference poses (Vicon raw poses transformed)

addpath('../functions/InLocCIIRC_utils/rotationDistance');
addpath('../functions/InLocCIIRC_utils/mkdirIfNonExistent');
addpath('../functions/InLocCIIRC_utils/load_CIIRC_transformation');
addpath('../functions/InLocCIIRC_utils/P_to_str');
addpath('../functions/InLocCIIRC_utils/R_to_numpy_array');
addpath('../functions/InLocCIIRC_utils/rotationMatrix');
addpath('../functions/InLocCIIRC_utils/params');
addpath('../functions/InLocCIIRC_utils/projectPointCloud');
[ params ] = setupParams('holoLens1'); % NOTE: tweak

projectPC = false; % NOTE: tweak

%% build HoloLens poses table w.r.t. to HoloLens CS
descriptionsTable = readtable(params.queryDescriptions.path); % decribes the reference poses
rawHoloLensPosesTable = readtable(params.holoLens.poses.path);
assert(size(descriptionsTable,1) == size(rawHoloLensPosesTable,1));
nQueries = size(descriptionsTable,1);

blacklistedQueries = false(1,nQueries);
blacklistedQueries(params.blacklistedQueryInd) = true;
nBlacklistedQueries = sum(blacklistedQueries);
fprintf('You have blacklisted %0.0f%% queries. %d queries remain.\n', ...
            nBlacklistedQueries*100/nQueries, nQueries-nBlacklistedQueries);
whitelistedQueries = logical(ones(1,nQueries) - blacklistedQueries); % w.r.t. reference frames

nPts = nQueries;
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
    R = rotmat(quaternion(orientation), 'frame'); % what are the columns of R? 
        % Bases of WHAT wrt WHAT? (one of them is initial unknown HL CS, the other is HL camera CS)
        % -> it is most likely a rotation matrix from initial unknown HL CS to HL camera CS. i.e. the columns
        % are bases of initial unknown HL CS in HL camera CS coordinates
        % Otherwise this code wouldn't work

    % camera points to -z in HoloLens
    % see https://docs.microsoft.com/en-us/windows/mixed-reality/coordinate-systems-in-directx
    rFix = rotationMatrix([pi, 0.0, 0.0], 'ZYX');
    R = rFix * R;

    P = eye(4);
    P(1:3,1:3) = R;
    P(1:3,4) = R * -t; % TODO: why is there the minus? actually, its useless. because the code actually extrects the t from P(1:3,4)
    Ps{i} = P;
    pts(idx,:) = t';
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
    R_ref = P_ref(1:3,1:3); % camera bases wrt model
    T_ref = -inv(R_ref)*P_ref(1:3,4); % wrt model
    
    pts_ref(idx,:) = T_ref';
    idx = idx + 1;
end

%% build HoloLens poses table w.r.t. to model CS

% due to a (possible) delay, we need to match frames between HoloLens and reference (Vicon)
nWhitelistedPts = sum(whitelistedQueries);
ptsWhitelisted_ref = zeros(nWhitelistedPts,3);
ptsWhitelisted = zeros(nWhitelistedPts,3);

idx = 1;
for i=1:nQueries
    if whitelistedQueries(i)
        ptsWhitelisted_ref(idx,:) = pts_ref(i,:);
        j = i + params.HoloLensTranslationDelay;
        if j > nQueries
            ptsWhitelisted(idx,:) = repmat(-1,1,3); % dummy, will be disregarded later
        else
            ptsWhitelisted(idx,:) = pts(j,:);
        end
        idx = idx + 1;
    end
end

pts = ptsWhitelisted(1:nWhitelistedPts-params.HoloLensTranslationDelay,:);
pts_ref = ptsWhitelisted_ref(1:nWhitelistedPts-params.HoloLensTranslationDelay,:);

A = eye(4);
[d,Z,transform] = procrustes(pts_ref, pts, 'scaling', true, 'reflection', false);
R = transform.T;
A(1:3,1:3) = R; % NOTE: first, R must be correct, then t can be correct
A(1:3,4) = -R*transform.c(1,:)';

Ts = repmat(-1, nQueries, 3);
for i=1:nQueries
    P = holoLensPosesTable.P{i};
    translationIdx = i + params.HoloLensTranslationDelay;
    orientationIdx = i + params.HoloLensOrientationDelay;
    if translationIdx > nQueries || orientationIdx > nQueries
        P = zeros(4); % dummy, will be disregarded later
    else
        translationP = holoLensPosesTable.P{translationIdx};
        orientationP = holoLensPosesTable.P{orientationIdx};
        t = -inv(translationP(1:3,1:3))*translationP(1:3,4);
        t = transform.b * t;
        t = [-orientationP(1:3,1:3)*t; 0];
        P = [orientationP(1:4,1:3), t];
        P = P * A; % why is this not A * P ??
    end
    holoLensPosesTable.P{i} = P;
    t = -inv(P(1:3,1:3))*P(1:3,4);
    Ts(i,:) = t;
end

%% store the HoloLens poses for future reference
mkdirIfNonExistent(params.HoloLensPoses.dir);
readmeFile = fopen(fullfile(params.HoloLensPoses.dir, 'readme.txt'), 'w');
fprintf(readmeFile, 'HoloLens poses w.r.t model (Matterport) CS.\nErrors are w.r.t. reference poses.');
fclose(readmeFile);
nQueriesWithoutEnd = nQueries-max([params.HoloLensTranslationDelay, params.HoloLensOrientationDelay]);
for i=1:nQueriesWithoutEnd
    id = holoLensPosesTable{i, 'id'};
    P = holoLensPosesTable.P{i};
    poseFile = fopen(fullfile(params.HoloLensPoses.dir, sprintf('%d.txt', id)), 'w');
    P_str = P_to_str(P);
    fprintf(poseFile, '%s', P_str);
    fclose(poseFile);
end

%% Evaluate w.r.t reference poses
% NOTE: For translation/orientation delay > 1, the last pose in reference frames does
% not have a matching HoloLens pose.
clearvars errors
errors(nQueries,1) = struct();
for i=1:nQueries % TODO: ugly init, due to my lack of MATLAB understanding
    id = holoLensPosesTable{i, 'id'};
    errors(i).queryId = id;
    errors(i).translation = -1; % this means we couldn't verify the,
    errors(i).orientation = -1; % correctness due to the delay
end

%%
Ts_ref = repmat(-1, nQueries, 3);
for i=1:nQueries
    if i <= nQueriesWithoutEnd
        id = holoLensPosesTable{i, 'id'};
        P = holoLensPosesTable.P{i};
        posePath = fullfile(params.poses.dir, sprintf('%d.txt', id));
        P_ref = load_CIIRC_transformation(posePath);
        T = -inv(P(1:3,1:3))*P(1:3,4);
        T_ref = -inv(P_ref(1:3,1:3))*P_ref(1:3,4);
        R = P(1:3,1:3);
        R_ref = P_ref(1:3,1:3);
        errors(i).translation = norm(T - T_ref);
        errors(i).orientation = rotationDistance(R_ref, R);
    else
        T_ref = zeros(3,1);
    end
    Ts_ref(i,:) = T_ref;
end

relevancyArray = logical(([errors.translation] ~= -1) .* whitelistedQueries);
relevantErrors = errors(relevancyArray);
avgTerror = mean(cell2mat({relevantErrors.translation}));
avgRerror = mean(cell2mat({relevantErrors.orientation}));
summaryMessage = sprintf('Mean errors (whitelist only): translation: %0.2f [m], orientation: %0.2f [deg]\n', avgTerror, avgRerror);
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
close all
tiledlayout(2,1);

nexttile
histogram(cell2mat({relevantErrors.translation}));
title('HoloLens to reference poses: Translation errors (whitelist only)');
xlabel('Translation error [m]');
ylabel('Number of occurences');

nexttile
maxValue = ceil(max(cell2mat({relevantErrors.orientation})));
histogram(cell2mat({relevantErrors.orientation}), 2*maxValue+1, 'BinWidth', 0.5);
xticks(0:1:maxValue)
title('HoloLens to reference poses: Orientation errors (whitelist only)');
xlabel('Orientation error [deg]');
ylabel('Number of occurences');

queryDirName = strsplit(params.dataset.query.dir, '/');
queryDirName = queryDirName{end};
filename = sprintf('errorDistribution-%s.pdf', queryDirName);
saveas(gcf, fullfile(params.HoloLensPoses.dir, filename));

figure();
Ts_ref = Ts_ref(relevancyArray,:);
Ts = Ts(relevancyArray,:);
Tdiffs = Ts - Ts_ref;
scatter3(Tdiffs(:,1), Tdiffs(:,2), Tdiffs(:,3));
title('Position diffs (whitelist only)');
xlabel('x');
ylabel('y');
zlabel('z');
filename = sprintf('positionDiffs-%s.pdf', queryDirName);
saveas(gcf, fullfile(params.HoloLensPoses.dir, filename));
TdiffsMedian = median(Tdiffs);
fprintf('Position diffs median (whitelist only): [%f, %f, %f]\n', ...
        TdiffsMedian(1), TdiffsMedian(2), TdiffsMedian(3)); % NOTE: not an absolute error
TdiffsCov = cov(Tdiffs)
TdiffsCor = corrcoef(Tdiffs)
    
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

    queryFilename = sprintf('%d.jpg', id);
    queryImg = imread(fullfile(params.dataset.query.dir, queryFilename));
    outQueryPath = fullfile(params.HoloLensProjectedPointCloud.dir, queryFilename);
    imwrite(queryImg, outQueryPath);
end