#!/bin/bash
#$ -cwd
#$ -o run_buildRawPoses_cmp.output.txt
#$ -e run_buildRawPoses_cmp.error.txt
#$ -pe smp 1
#$ -q offline@cmpgrid-65

date
cd "/mnt/datagrid/personal/lucivpav/InLocCIIRC_dataset (repo)/query"
ml load MATLAB/9.7
cat buildRawPoses.m | matlab -nodesktop
date
