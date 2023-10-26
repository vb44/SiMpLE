# SiMpLE LiDAR odometry
Simple Mapping and Localisation Estimation (SiMpLE) is a low-drift and low-configuration LiDAR odometry method providing accurate localisation and mapping with a variety of sensors.

[Click here for a demo!](https://github.com/vb44/SiMpLE/assets/63623876/1a9ef9ed-845d-4724-9c16-b487c8e77081)

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
The method has four configuration parameters.
1. ***rNew*** is the new point cloud spatial subsampling radius.
2. ***rMap*** is the spatial subsampling radius for the local map.
3. ***&sigma;*** is the standard deviation of the Gaussian reward function - see the paper for more details.
4. ***&epsilon;*** is the convergence tolerance of the optimisation solver. 

There are two more optional settings:
1. ***rMax*** is the maximum radius of the point cloud. This is hardware-dependent and not a configuration parameter.
2. ***rMin*** is for computational benefit only. The default value is zero where it has no effect. Increasing this value eliminates the rings commonly formed around the sensor to reduce the size of the point cloud without losing geometric information. 

## Hardware and Dependencies
This implementation has been tested on Ubuntu 20.04.5 LTS (Focal Fossa) with an Intel Core i7-10700K CPU @ 3.80GHz x 16 and 62.5 GiB memory.
SiMpLE uses a few open-source libraries for Kd-Trees, matrix operations, optimisation functions, and CPU threading.
The installation instructions are detailed below.

* Git is required to download opensource libraries.
```bash
sudo apt install git
```
* Require a g++ compiler.
```bash
sudo apt install g++
```
* CMake is required to compile the libraries and the repository.
```bash
sudo apt install cmake
```
* Install the Eigen library for math operations [1].
```bash
sudo apt install libeigen3-dev
```
* Install Intel's Thread Building Blocks (TBB) library for CPU threading.
```bash
sudo apt install libtbb-dev
```
* Clone and install the nanoflann library for KD-tree operations [2].
```bash
git clone https://github.com/jlblancoc/nanoflann.git
cd nanoflann
mkdir build && cd build
cmake ..
sudo make install
```
* Install the *Dlib* library for optimisation solvers [3, 4].
```bash
wget http://dlib.net/files/dlib-19.24.tar.bz2
tar xvf dlib-19.24.tar.bz2
cd dlib-19.24/
mkdir build && cd build
cmake ..
cmake --build . --config Release
sudo make install
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
cd SiMpLE
mkdir build && cd build
```

Run CMake.
```bash
cmake ../src
```

Make the executable.
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
--kitti (optional)
--verbose (optional)
```
The verbose mode prints information to the terminal.
Sample use with the KITTI dataset is shown below.\
***Note that the --kitti argument is required to correct the scans for the KITTI dataset.*** 

```bash
./simple --path KITTI/07/velodyne/ --sigma 1.0 --rMap 2.0 --rNew 1.0 --convergenceTolerance 1e-6 --minSensorRange 0 --maxSensorRange 120 --outputFileName ./results/test_Kitti_07 --kitti
```

## Sample results interpretation
An example of interpreting the result file is displayed in *plotResultsMATLAB/interpretResults.m*.

## Other datasets
The UQ dataset scans can be downloaded from https://drive.google.com/drive/folders/1JUdrDOG3lWwZEgmuJvowtxrWuK0s2sZ3?usp=drive_link.

## References
[1] Eigen library: https://eigen.tuxfamily.org/dox/GettingStarted.html \
[2] Nanoflann library: https://github.com/jlblancoc/nanoflann \
[3] Dlib library: http://dlib.net/compile.html \
[3] Dlib C++ install for CMake: https://learnopencv.com/install-dlib-on-ubuntu/