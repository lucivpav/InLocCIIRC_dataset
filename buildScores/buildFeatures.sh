#!/bin/bash
nvidia-smi --query-gpu=index,name,utilization.memory --format=csv
echo -n "Please select a GPU: "
read GPU_ID
export CUDA_VISIBLE_DEVICES=$GPU_ID
module load MATLAB/9.4
cat buildFeatures.m | matlab -nodesktop 2>&1 | tee buildFeatures.log