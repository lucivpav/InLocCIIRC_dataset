function evaluateMatchesUsingPs(queryInd, params, Rs, ts)

    nQueries = size(queryInd,2);
    error = zeros(nQueries,1);
    for i=1:nQueries
        queryIdx = queryInd(i);
        thisInterestingPointsPC = params.interestingPointsPC{queryIdx};
        P = [params.K*Rs{i}, -params.K*Rs{i}*ts{i}];
        projectedInterestingPoints{i} = projectPointsUsingP(thisInterestingPointsPC, P);
        thisInterestingPointsQuery = params.interestingPointsQuery{queryIdx};
        error(i) = norm(thisInterestingPointsQuery - projectedInterestingPoints{i});
    end
    
    for i=1:size(error)
        fprintf('Interesting query %d error: %0.2f\n', queryInd(i), error(i));
    end
    fprintf('Error sum: %0.2f\n', sum(error,1));
                                        
    %% visualize correspondences and errors
    for i=1:nQueries
        queryIdx = queryInd(i);

        figure;
        pointSize = 8.0;
        outputSize = params.camera.sensor.size;
        projectedPointCloud = projectPointCloud(params.pointCloud.path, params.camera.fl, Rs{i}, ...
                                            ts{i}, params.camera.sensor.size, outputSize, pointSize, ...
                                            params.projectPointCloudPy.path);
        image(projectedPointCloud);
        axis image;
    
        hold on;
        thisProjectedInterestingPoints = projectedInterestingPoints{i} + 1; % MATLAB is 1-based
        thisInterestingPointsQuery = params.interestingPointsQuery{queryIdx} + 1; % MATLAB is 1-based
        scatter(thisProjectedInterestingPoints(1,:), thisProjectedInterestingPoints(2,:), 40, 'r', 'filled');
        scatter(thisInterestingPointsQuery(1,:), thisInterestingPointsQuery(2,:), 40, 'g', 'filled');
        nCorrespondences = size(thisInterestingPointsQuery,2);
        for i=1:nCorrespondences
            plot([thisInterestingPointsQuery(1,i), thisProjectedInterestingPoints(1,i)], ...
                 [thisInterestingPointsQuery(2,i), thisProjectedInterestingPoints(2,i)], ...
                 'r-', 'linewidth', 2);
        end
        hold off;
        set(gcf, 'Position', get(0, 'Screensize'));
    end
    
    end