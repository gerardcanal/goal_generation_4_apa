#!/bin/bash

TMPDIR="/tmp/k1811650/stmp__$(date +%d_%m_%y_%s_%N)"
mkdir -p $TMPDIR
echo "  Container using $TMPDIR as /tmp"
CONTAINER_FOLDER="/data/data/gerard/containers"
singularity run --net --network none -B /data/data/gerard -B $TMPDIR:/tmp $CONTAINER_FOLDER/ortools.sif "$@"
# With bin bind singularity run --net --network none -B /data/data/gerard -B /tmp/k1811650/bin:/data/data/gerard/goal_generation_4_apa/bin -B $TMPDIR:/tmp $CONTAINER_FOLDER/ortools.sif "$@"
#singularity run --net --network none -B $TMPDIR:/tmp /home/gerard/Desktop/ortools.sif "$@"
