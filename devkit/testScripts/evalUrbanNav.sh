# !/bin/bash

estFilePath=../../sampleResults/UrbanNav/tumFormat

evo_rpe tum ../../sampleResults/UrbanNav/groundTruth/novatel_data2_0428.txt ${estFilePath}/TUM_2019_04_28_20_58_02.txt
evo_rpe tum ../../sampleResults/UrbanNav/groundTruth/novatel_data2_0428.txt ${estFilePath}/TUM_2019_04_28_20_58_02.txt --pose_relation angle_deg

evo_rpe tum ../../sampleResults/UrbanNav/groundTruth/novatel_data1_0314.txt ${estFilePath}/TUM_2020_03_14_16_45_35.txt
evo_rpe tum ../../sampleResults/UrbanNav/groundTruth/novatel_data1_0314.txt ${estFilePath}/TUM_2020_03_14_16_45_35.txt --pose_relation angle_deg