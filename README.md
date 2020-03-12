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

## Obtain the sweepData.json file
1. Run a web server in the getSweepData folder
2. Open the getSweepData.html in your browser **as a localhost** address - e.g. http://127.0.0.1:8887/getSweepData.html
3. Open the console, the sweepData.json is being printed there

## Obtain panoramas
* Manually download a panorama for every sweep
* Name the panoramas according to their number as taken by the Capture iPadOS app
* Make sure the circle around the mouse pointer is not present in the panorama
* Name the panoramas as *number*.pano in matterport.com
* Download the panoramas as *number.jpg*

## Obtain MatterPak and normalize it
1. Buy the MatterPak
2. Download it, it contains cloud.xyz, .obj files
3. Rotate them along the x axis (psi angle) by -90.0 degrees
4. Save them accordingly into the models directory, use .ply extension

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

## Build transformations
1. Adjust the Space name

## Build point cloud file
1. Adjust the Space name

## Build query poses
1. Adjust the Space name

## Build file lists
1. Note the comment on the second line

## Build scores
1. Execute buildFeatures.m on a machine with GPU
2. Execute buildScores on a machine with ~40 GB of RAM

## TODO
* The mesh should be in .obj format -> update the README and code. why .obj? to preserve high quality RGB triangulated model
* Make the scores more relevant. Right now they don't represent the query-cutouts very well.