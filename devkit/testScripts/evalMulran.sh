# !/bin/bash

# Evaluate the MulRan results.
gtFilePath=../../sampleResults/MulRan/groundTruthInSensorFrame/
estFilePath=../../sampleResults/MulRan/simpleOutputs/

for test in DCC_01 DCC_02 DCC_03 KAIST_01 KAIST_02 KAIST_03 Riverside_01 Riverside_02 Riverside_03 Sejong_01 Sejong_02 Sejong_03
do
    ../build/evaluate_odometry ${gtFilePath}/${test}.txt ${estFilePath}/${test}
done