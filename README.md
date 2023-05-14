# SiMpLE LiDAR odometry
Simple Mapping and Localisation Estimation (SiMpLE) is a low-drift and low-configuration LiDAR odometry method providing accurate localisation and mapping with a variety of sensors.

[Click here for a demo!](https://github.com/vb44/SiMpLE/assets/63623876/d48e96bc-8172-4522-9099-de60d729ecf3)

We perform among state-of-the-art LiDAR odometry methods with a simple and use case agnostic algorithm.
The method is derived from using raw point clouds. We do not modify or develop a new variant of any existing algorithm.
The code is written for readability and easy understanding with the methodology described in the paper.
The software can be optimised for increased performance.
<!-- Our paper is available at https://www.paperLink.com. -->

We have tested the algorithm using LiDARs with various point cloud densities, fields of view, and scan patterns,
* Velodyne HDL-64 KITTI dataset 
* OS1-64 from the MulRan dataset
* Velodyne VLP-16 using a self-created dataset
* Livox Horizon using a self-created dataset

The motivation stems from reducing the complexity and configuration burden of localisation and mapping algorithms.

The code is portable, easy to understand, and modify.

## Method
The method has three steps and four configuration parameters displayed in green.
![methodology](media/methodology_green.jpg)

The three steps:
1. Spatially subsample the input scan at a configurable radius, *rNew*.
2. Register the subsampled input scan to the local map using the configurable parameters &sigma;, and &epsilon;.
2. Update the local map at a spatial subsampling radius of *rMap*.

The four configuration parameters:
1. ***rNew*** is the new point cloud spatial subsampling radius.
2. ***rMap*** is the spatial subsampling radius for the local map.
3. ***&sigma;*** is the standard deviation of the Gaussian reward function - see the paper for more details.
4. ***&epsilon;*** is the convergence tolerance of the optimisation solver. 

There are two more optional settings:
1. ***rMax*** is the maximum radius of the point cloud. This is hardware dependant and not a configuration parameter.
2. ***rMin*** is for computational benefit only. The default value is zero where it has no effect. Increasing this value eliminates the rings commonly formed around the sensor to reduce the size of the point cloud without losing geometric information. 

## Dependencies
SiMpLE uses a few open-source libraries for Kd-Trees, matrix operations, optimisation functions, and CPU threading.
* The *nanoflann* library for KD-tree operations: https://github.com/jlblancoc/nanoflann
* The *Eigen* library for maths operations: https://eigen.tuxfamily.org/dox/GettingStarted.html
* The *Dlib* library for optimisation solvers: http://dlib.net/compile.html
* Intel's *Thread Building Blocks (TBB)* for CPU threading: 
```bash
sudo apt install libtbb-dev 
```
Alternative options for any of the libraries can be used if desired.
The code is easy to change.

## Installation
Clone the repository.
```bash
git clone https://github.com/vb44/SiMpLE.git
```

Create a build folder in the repository.
```bash
mkdir build && cd build
```

Run CMake.
```bash
cmake ../src
```

Make the executables.
```bash
make
```

## Example
<!-- Show example usage. -->
Only works with *.bin* files in the KITTI format.
However, the code is human-friendly and very easy to modify to suit the desired inputs and outputs.
When compiled, the SiMpLE algorithm is run from the command line as shown below.
```bash
./simple 
--path                    "path_to_scans" 
--sigma                   "sigma_value" 
--rMap                    "radius [m]" 
--rNew                    "radius [m]" 
--convergenceTolerance    "tolerance" 
--maxSensorRange          "radius [m]" 
--minSensorRange          "radius [m]" 
--outputFileName          "fileName" 
--verbose (optional)
```
The verbose mode prints information to the terminal.
Sample use with the KITTI dataset:

```bash
./simple --path KITTI/07/velodyne/ --sigma 1.0 --rMap 2.0 --rNew 1.0 --convergenceTolerance 1e-6 --minSensorRange 0 --maxSensorRange 120 --outputFileName ./results/test_Kitti_07
```

## Sample results interpretation
An example of interpreting the result file is displayed in *plotResultsMATLAB/interpretResults.m*.