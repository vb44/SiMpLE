# Evaluating KITTI and MulRan Datasets

The devkit provided here is a simplified version of KITTI's devkit available at https://www.cvlibs.net/datasets/kitti/eval_odometry.php.\
The tool takes two inputs, (1) the path to the ground truth file, and (2) the path to the trajectory estimate.\
The output is the average translational error (%) and the rotational error (deg/100m).

## Build
```bash
cd SiMpLE/devkit/
mkdir build && cd build
cmake ../src
```

## Usage
The usage is as follows,
```bash
./eval_odometry gtFile estFile
```
Both *gtFile* and *estFile* are in the KITTI format.

## Test scripts
The *testScripts* folder has sample scripts for evaluating the files in *sampleResults*.\
The bash scripts can be easily modified to different trajectory estimate paths.\
Sample outputs from the files are displayed below.
```bash
# KITTI sample results.
./evalKitti.sh

# Output for seq 00-10 (average translational error %, rotational error deg/100m).
0.746776, 0.409515
0.606345, 0.13557
0.692663, 0.248021
0.741957, 0.170942
0.396594, 0.168461
0.389486, 0.327608
0.275149, 0.130415
0.586706, 0.623548
0.836805, 0.270701
0.624387, 0.274239
0.779416, 0.423699

# MulRan sample results.
./evalMulran.sh

# Output for all 12 seq (average translational error %, rotational error deg/100m).
2.68703, 0.666249
2.08788, 0.520731
1.81469, 0.575992
2.18896, 0.59395
2.08484, 0.610183
2.39273, 0.67435
3.15843, 0.606495
2.99325, 0.616493
2.35602, 0.607249
5.26479, 0.506253
4.29148, 0.557695
5.29888, 0.725139
```