This repository contains tools for building the InLocCIIRC dataset.
The dataset is constructed incrementally for each Space.
The usual steps are as follows:

1. Obtain the sweepData.json file
2. Obtain panoramas
3. Obtain MatterPak and normalize it
4. Rotate panoramas
5. Build cutouts
6. Build transformations
7. Build point cloud file
8. Build query poses
9. Build file lists
10. Build scores
11. Plot the dataset including retrieved poses
12. Plot query pipeline

## Obtain the sweepData.json file
1. Create a key.js file based on the keyTemplate.js file
2. Run a web server in the getSweepData folder
3. Open the getSweepData.html in your browser **as a localhost** address - e.g. http://127.0.0.1:8887/getSweepData.html
4. Open the console, the sweepData.json is being printed there

## Obtain panoramas
* Manually download a panorama for every sweep
* Name the panoramas according to their number as taken by the Capture iPadOS app
* Make sure the circle around the mouse pointer is not present in the panorama
* Name the panoramas as *number*.pano in matterport.com
* Download the panoramas as *number.jpg*

## Obtain MatterPak and normalize it
1. Buy the MatterPak
2. Download it, it contains cloud.xyz, .obj files
3. Rotate them along the x axis (psi angle) by -90.0 degrees; recommended tool: CloudCompare
4. Save them accordingly into the models directory, use .ply extension for the point cloud, .obj extension for the mesh

## Rotate panoramas
1. Open the rotatePanoramas folder in Matlab
2. Set up the appropriate Space name in setupParams.m
3. Adjust and run buildSweepDataMatFile.m
4. Adjust and run rotatePanoramas.m
5. For panoramas that failed to rotate properly, try changing the *goodness* in sweepData
6. Try increasing the point size of the point cloud projection
7. If the proper rotation still cannot be found, use manuallyRotatePanorama.m file

## Build cutouts
1. Adjust the Space name and the panoIds array
2. It is necessary that the display is turned on, otherwise you get an error from pyrender

## Build transformations
1. Adjust the Space name

## Build point cloud file
1. Adjust the Space name

## Build query poses
1. No manual intervention necessary

## Build file lists
1. Note the comment on the second line

## Build scores
1. Execute buildFeatures.m on a machine with GPU
2. Execute buildScores on a machine with ~40 GB of RAM

## Plot the dataset including retrieved poses 
1. Make sure the demo has finished and now we have retrievedPoses.csv in evaluation directory
2. It is recommended to erase evaluation/temporary directory
3. Run evaluation/spaceTopViews.py and check that the output images are looking good

## Plot query pipeline
1. Run evaluation/queryPipeline.m for queries of interest

## TODO
* Make the scores more relevant. Right now they don't represent the query-cutouts very well.