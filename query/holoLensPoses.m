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
    R = quat2rotm(orientation); % from docs: "When using the rotation matrix, premultiply it with the coordinates to be rotated (as opposed to postmultiplying)."
    P = eye(4);
    P(1:3,1:3) = R;
    P(1:3,4) = R * -t;
    Ps{i} = P;
    xxx = 1;
end
Ps = {Ps};
Ps = Ps{1,1};
Ps = reshape(Ps, nQueries, 1);
ids = {descriptionsTable.id};
ids = ids{1,1};
ids = reshape(ids, nQueries, 1);
holoLensPosesTable = table(ids, Ps);
holoLensPosesTable.Properties.VariableNames = {'id', 'P'};

%% build HoloLens poses table w.r.t. to model CS
% this should be a query Id for which we have a pretty correct reference pose and for which the HoloLens pose is pretty correct
goodQueryId = 1;
P_ref = load_CIIRC_transformation(fullfile(params.poses.dir, sprintf('%d.txt', goodQueryId)));
goodIdx = find(holoLensPosesTable{:, 'id'} == goodQueryId);
P = holoLensPosesTable{goodIdx, 'P'}{1,1};
A = P_ref * inv(P); % transforms P from HoloLens CS to model (Matterport) CS

for i=1:nQueries
    P = holoLensPosesTable.P{i};
    P = A * P;
    holoLensPosesTable.P{i} = P;
end

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

%% write errors to file
for i=1:nQueries
    translation2{i} = sprintf('%0.2f', errors(i).translation);
    orientation2{i} = sprintf('%0.2f', errors(i).orientation);
end
errorsTable = table({errors.queryId}', translation2', orientation2');
errorsTable.Properties.VariableNames = {'id', 'translation', 'orientation'};
errorsPath = fullfile(params.HoloLensPoses.dir, 'errors.csv');
writetable(errorsTable, errorsPath);