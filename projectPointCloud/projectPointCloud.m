function projection = projectPointCloud(pcPath, f, R, t, sensorSize, outputSize, pointSize, ...
                                        projectPointCloudPyPath)

inputPath = strcat(tempname, '.mat');
outputPath = strcat(tempname, '.jpg');
save(inputPath, 'pcPath', 'f', 'R', 't', 'sensorSize', 'outputSize', 'pointSize');

% call projectPointCloud.py
command = sprintf('PATH=/usr/local/bin:$PATH python3 %s %s %s', projectPointCloudPyPath, inputPath, outputPath);
disp(command)
[status, cmdout] = system(command);
disp(cmdout)

% load results
projection = imread(outputPath);

% delete temporary files
delete(inputPath);
delete(outputPath);

end
