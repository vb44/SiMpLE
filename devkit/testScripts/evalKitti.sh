# !/bin/bash

# Evaluate the KITTI results.
gtFilePath=../../sampleResults/KITTI/groundTruth/
estFilePath=../../sampleResults/KITTI/estimatesInGroundTruthFrame/online/
# estFilePath=../../sampleResults/KITTI/estimatesInGroundTruthFrame/offline/

for test in 00 01 02 03 04 05 06 07 08 09 10
do
    ../build/evaluate_odometry ${gtFilePath}/${test}.txt ${estFilePath}/${test}.txt
done