# SiMpLE LiDAR odometry
Simple Mapping and Localisation Estimation (SiMpLE) is a low-drift and low-configuration LiDAR odometry method providing accurate localisation and mapping with a variety of sensors.

[Click here for a demo!](https://github.com/vb44/SiMpLE/assets/63623876/5fc7663b-f255-4b8c-a57b-f8288e56439e)

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

The motivation stems from reducing the complexity and configuration burden of localisation algorithms.

The code is portable, easy to understand, and modify.

## Method
The method has four configuration parameters.
1. ***rNew*** is the new point cloud spatial subsampling radius.
2. ***rMap*** is the spatial subsampling radius for the local map.
3. ***&sigma;*** is the standard deviation of the Gaussian reward function - see the paper for more details.
4. ***&epsilon;*** is the convergence tolerance of the optimisation solver. 

There are two more settings:
1. ***rMax*** is the maximum radius of the point cloud. This is hardware-dependent and not a configuration parameter.
2. ***rMin*** is optional and for computational benefit only. The default value is zero where it has no effect. Increasing this value eliminates the rings commonly formed around the sensor to reduce the size of the point cloud without losing geometric information. 

## Hardware and Dependencies
This implementation has been tested on Ubuntu 20.04.5/6 LTS (Focal Fossa) and Ubuntu 22.04.3 (Jammy Jellyfish) with an Intel Core i7-10700K CPU @ 3.80GHz x 16 and 62.5 GiB memory.
SiMpLE uses a few open-source libraries for Kd-Trees, matrix operations, optimisation functions, and CPU threading.
The installation instructions are detailed below.

* Git is required to download the open-source libraries.
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
If the SiMpLE repository build in the following section returns an error that it cannot find TBB for CMake, the following installation may help.
```bash
git clone https://github.com/oneapi-src/oneTBB
cd oneTBB
mkdir build && cd build
cmake ..
sudo make install
```
* Clone and install the *nanoflann* library for KD-tree operations [2].
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
./simple --path KITTI/07/velodyne/ --sigma 0.3 --rMap 2.0 --rNew 0.35 --convergenceTolerance 1e-4 --minSensorRange 10 --maxSensorRange 80 --outputFileName ./results/test_Kitti_07 --kitti
```
## Sample results interpretation
An example of interpreting the result file in the KITTI format is displayed in *plotResultsMATLAB/interpretResults.m*.

## Sample results and evaluation
Sample results from the paper are available in the ***sampleResults*** folder for all reported datasets.\
The estimated trajectories can be evaluated in the *devkit* folder, which is a reduced version of the devkit provided by KITTI.

## Results on different hardware and software versions
Slight variations were observed when executing the algorithm on different machines.\
This is caused by the Dlib optimisation library having slight numerical precision differences when searching for the best pose hypothesis.
* Ubuntu 20.04.5: Desktop with Intel Core i7 (10th gen, 16 cores).
* Ubuntu 20.04.6: Laptop with Intel Core i7 (8 cores).
* Ubuntu 22.04.3: Desktop virtual machine with Intel Core i7 (10th gen, 10 cores).
* Fedora 38     : Laptop with Intel Core i7 (9th gen, 12 cores).

The KITTI results on different machines are shown below.

|    Sequence | Ubuntu 20.04.5 | Ubuntu 20.04.6 | Ubuntu 22.04.3 | Fedora 38  |
|------------:|----------------|----------------|----------------|------------|
|      **00** |     0.7468     |     0.7468     |     0.7473     |   0.7507   |
|      **01** |     0.6063     |     0.6063     |     0.6062     |   0.6169   |
|      **02** |     0.6913     |     0.6927     |     0.6914     |   0.6671   |
|      **03** |     0.7420     |     0.7420     |     0.7420     |   0.7270   |
|      **04** |     0.3966     |     0.3966     |     0.3966     |   0.3987   |
|      **05** |     0.3922     |     0.3895     |     0.3873     |   0.3817   |
|      **06** |     0.2751     |     0.2751     |     0.2751     |   0.2796   |
|      **07** |     0.5867     |     0.5867     |     0.5867     |   0.5830   |
|      **08** |     0.8392     |     0.8368     |     0.8392     |   0.8647   |
|      **09** |     0.6244     |     0.6244     |     0.6244     |   0.5996   |
|      **10** |     0.7794     |     0.7794     |     0.7680     |   0.8309   |
| **avg (%)** |   **0.6073**   |   **0.6069**   |   **0.6058**   | **0.6091** |

## Other datasets
The UQ dataset scans can be downloaded from https://drive.google.com/drive/folders/1PgECQIySs72Qz-CXhAsAKSq_fRi5S3oA?usp=sharing.
The folder contains the raw VLP-16 and Livox scans in the KITTI format for comparison with the sensor trajectory results shown in the paper.

## References
[1] Eigen library: https://eigen.tuxfamily.org/dox/GettingStarted.html \
[2] Nanoflann library: https://github.com/jlblancoc/nanoflann \
[3] Dlib library: http://dlib.net/compile.html \
[3] Dlib C++ install for CMake: https://learnopencv.com/install-dlib-on-ubuntu/
