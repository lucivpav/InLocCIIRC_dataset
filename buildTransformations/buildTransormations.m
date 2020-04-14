addpath('../functions/InLocCIIRC_utils/rotationMatrix');
addpath('../functions/local/P_to_str')

params = struct();
params.dataset.dir = '/Volumes/GoogleDrive/Můj disk/ARTwin/InLocCIIRC_dataset';
params.spaceName = 'B-315';
params.sweepData.mat.path = fullfile(params.dataset.dir, 'sweepData', sprintf('%s.mat', params.spaceName));
params.alignments.dir = fullfile(params.dataset.dir, 'alignments', params.spaceName);
params.all_transformations.path = fullfile(params.alignments.dir, 'all_transformations.txt');
params.known_incorrect.path = fullfile(params.alignments.dir, 'know_incorrect.txt');
params.transformations.dir = fullfile(params.alignments.dir, 'transformations');
params.f = 1385.6406460551023;
params.sensorSize = [1600 1200];

if exist(params.alignments.dir, 'dir') ~= 7
    mkdir(params.alignments.dir);
end

if exist(params.transformations.dir, 'dir') ~= 7
    mkdir(params.transformations.dir);
end

load(params.sweepData.mat.path);
all_transformationsFile = fopen(params.all_transformations.path, 'w');

% create an empty file
knownIncorrectFile = fopen(params.known_incorrect.path, 'w');
fclose(knownIncorrectFile);

K = eye(3);
% K(1,1) = params.f;
% K(2,2) = params.f;
% K(1,3) = params.sensorSize(1)/2;
% K(2,3) = params.sensorSize(2)/2;

for i=1:size(sweepData, 2)
    sweepRecord = sweepData(i);
    thisPanoTransformationPath = fullfile(params.transformations.dir, sprintf('trans_%d.txt', sweepRecord.panoId));
    thisTransformationFile = fopen(thisPanoTransformationPath, 'w');
    angles = sweepRecord.rotation*pi/180;
    R = rotationMatrix(angles, 'XYZ'); % ZYX?
    P = eye(4);
    P(1:3,1:4) = K * [R sweepRecord.position];
    P_str = P_to_str(P);
    fprintf(all_transformationsFile, '%d\n\n%s\n', sweepRecord.panoId, P_str);
    fprintf(thisTransformationFile, '%s\n', P_str);
    fclose(thisTransformationFile);
end

fclose(all_transformationsFile);