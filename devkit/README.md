# Evaluating Public Datasets

The devkit provided here is a simplified version of KITTI's devkit available at https://www.cvlibs.net/datasets/kitti/eval_odometry.php.
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
# KITTI sample online results.
./evalKitti.sh

# Output for seq 00-10 online (average translational error %, rotational error deg/100m).
0.666664, 0.344745
0.774891, 0.112796
0.626282, 0.205304
0.736941, 0.139842
0.406705, 0.163427
0.337172, 0.280255
0.261315, 0.0986968
0.470648, 0.444404
0.823635, 0.220979
0.568401, 0.178258
0.645373, 0.35557

# KITTI sample offline results.
./evalKitti.sh

# Output for seq 00-10 offline (average translational error %, rotational error deg/100m).
0.51142, 0.180085
0.858036, 0.091102
0.525836, 0.132351
0.697397, 0.135559
0.396662, 0.104197
0.296488, 0.134704
0.277907, 0.0762397
0.299288, 0.182178
0.803959, 0.162351
0.54848, 0.118674
0.505588, 0.154473

# MulRan sample results.
./evalMulran.sh

# Output for all 12 seq (average translational error %, rotational error deg/100m).
2.63448, 0.641646
2.05077, 0.50786
1.77898, 0.562839
2.16955, 0.576765
2.05967, 0.592993
2.3919, 0.664801
3.07242, 0.597147
2.89034, 0.581182
2.17944, 0.548846
4.2136, 0.472079
4.7229, 0.519647
5.15986, 0.685484

# UrbanNav sample results.
./evalUrbanNav

# Output for both sequences (Data1 and Data2).
RPE w.r.t. translation part (m)
for delta = 1 (frames) using consecutive pairs
(not aligned)

       max      1.454892
      mean      0.174790
    median      0.165465
       min      0.001924
      rmse      0.243834
       sse      28.895068
       std      0.170010

RPE w.r.t. rotation angle in degrees (deg)
for delta = 1 (frames) using consecutive pairs
(not aligned)

       max      3.290050
      mean      0.301293
    median      0.163876
       min      0.002678
      rmse      0.510445
       sse      126.629289
       std      0.412039

RPE w.r.t. translation part (m)
for delta = 1 (frames) using consecutive pairs
(not aligned)

       max      0.647709
      mean      0.239104
    median      0.232679
       min      0.004927
      rmse      0.262216
       sse      20.420903
       std      0.107640

RPE w.r.t. rotation angle in degrees (deg)
for delta = 1 (frames) using consecutive pairs
(not aligned)

       max      5.933351
      mean      0.846342
    median      0.609587
       min      0.000994
      rmse      1.177413
       sse      411.731648
       std      0.818540
```