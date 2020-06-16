function [measurementTable, queryTable, queryFiles] = initiMeasurementAndQueryTables(params)
    measurementTable = readtable(params.measurement.path);
    measurementTable.Properties.VariableNames = {'frameNumber', 'FPS', 'marker', 'invalid', 'x', 'y', 'z', 'alpha', 'beta', 'gamma'};

    FPS = 100; % viz each row. TODO: assert

    measurementTable.frameNumber = measurementTable.frameNumber - measurementTable.frameNumber(1);
    measurementTable.timestampMs = measurementTable.frameNumber * (1 / FPS) * 1000;
    measurementTable = removevars(measurementTable, {'FPS', 'marker', 'frameNumber'});
    measurementTable = measurementTable(~measurementTable.invalid, {'timestampMs', 'x', 'y', 'z', 'alpha', 'beta', 'gamma'});

    queryFiles = dir(params.input.query.dir);
    queryFiles = queryFiles(endsWith({queryFiles.name}, '.jpg'));

    timestamps = {queryFiles.name};
    timestamps = extractBetween(timestamps, 1, strlength(timestamps)-4);
    timestamps = strcat('uint64(', timestamps, ')');
    timestamps = str2num(str2mat(timestamps));
    timestamps = (timestamps - timestamps(1)) / 10000;
    queryTable = table({queryFiles.name}', timestamps);
    queryTable.Properties.VariableNames = {'name', 'timestampMs'};
end