function projection = projectPointCloud(pcPath, f, R, t, sensorSize, outputSize, pointSize, ...
                                        projectPointCloudPyPath)

inputPath = strcat(tempname, '.mat');
outputPath = strcat(tempname, '.png');
save(inputPath, 'pcPath', 'f', 'R', 't', 'sensorSize', 'outputSize', 'pointSize');

% call projectPointCloud.py
command = sprintf('PATH=$PATH:/usr/local/bin python3 %s %s %s', projectPointCloudPyPath, inputPath, outputPath)
[status, cmdout] = system(command);
cmdout

% load results
projection = imread(outputPath);

% delete temporary files
delete(inputPath);
delete(outputPath);

end
