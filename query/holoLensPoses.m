% 1. Convert the HoloLens poses to poses w.r.t. the model
% 2. Evaluate the transfmored poses w.r.t. reference poses (Vicon raw poses transformed)

addpath('../functions/local/projectPointCloud');
addpath('../functions/InLocCIIRC_utils/rotationDistance');
addpath('../functions/InLocCIIRC_utils/mkdirIfNonExistent');
addpath('../functions/InLocCIIRC_utils/load_CIIRC_transformation');
addpath('../functions/InLocCIIRC_utils/P_to_str');
addpath('../functions/local/R_to_numpy_array');
[ params ] = setupParams('holoLens1Params');

%% build HoloLens poses table w.r.t. to HoloLens CS
descriptionsTable = readtable(params.queryDescriptions.path); % decribes the reference poses
rawHoloLensPosesTable = readtable(params.input.poses.path);
assert(size(descriptionsTable,1) == size(rawHoloLensPosesTable,1));
nQueries = size(descriptionsTable,1);
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
    Ts{i} = t;
end
Ts = reshape(Ts, nQueries, 1);
Ps = {Ps};
Ps = Ps{1,1};
Ps = reshape(Ps, nQueries, 1);
ids = {descriptionsTable.id};
ids = ids{1,1};
ids = reshape(ids, nQueries, 1);
holoLensPosesTable = table(ids, Ps);
holoLensPosesTable.Properties.VariableNames = {'id', 'P'};

%% extract reference poses
for i=1:nQueries
    id = holoLensPosesTable{i, 'id'};
    P_ref = load_CIIRC_transformation(fullfile(params.poses.dir, sprintf('%d.txt', id)));
    T_ref = -inv(P_ref(1:3,1:3))*P_ref(1:3,4);
    Ts_ref{i} = T_ref;
end
Ts_ref = reshape(Ts_ref, nQueries, 1);

%% build HoloLens poses table w.r.t. to model CS
A = eye(4);
X = [Ts_ref{:,1}]';
Y = [Ts{:,1}]';
[d,Z,transform] = procrustes(X,Y, 'scaling', false, 'reflection', false);
R = transform.T';
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
errors = struct();
for i=1:nQueries
    id = holoLensPosesTable{i, 'id'};
    P = holoLensPosesTable.P{i};
    P_ref = load_CIIRC_transformation(fullfile(params.poses.dir, sprintf('%d.txt', id)));
    T = -inv(P(1:3,1:3))*P(1:3,4);
    T_ref = -inv(P_ref(1:3,1:3))*P_ref(1:3,4);
    R = P(1:3,1:3);
    R_ref = P_ref(1:3,1:3);
    errors(i).queryId = id;
    errors(i).translation = norm(T - T_ref);
    errors(i).orientation = rotationDistance(R_ref, R);
end

% NOTE: some reference poses are wrong due to Vicon error, blacklist them
blacklistedQueryInd = [103:109, 162, 179:188, 191:193, 286:288];
blacklistedQueries = false(1,nQueries);
blacklistedQueries(blacklistedQueryInd) = true;
whitelistedQueries = logical(ones(1,nQueries) - blacklistedQueries);

avgTerror = mean(cell2mat({errors(whitelistedQueries).translation}));
avgRerror = mean(cell2mat({errors(whitelistedQueries).orientation}));
fprintf('Mean errors (whitelist only): translation: %0.2f [m], orientation: %0.f [deg]\n', avgTerror, avgRerror);

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
histogram(cell2mat({errors(whitelistedQueries).translation}));
title('HoloLens to Matterport poses: Translation errors (whitelist only)');
xlabel('Translation error [m]');
ylabel('Number of occurences');

nexttile
histogram(cell2mat({errors(whitelistedQueries).orientation}));
title('HoloLens to Matterport poses: Orientation errors (whitelist only)');
xlabel('Orientation error [deg]');
ylabel('Number of occurences');