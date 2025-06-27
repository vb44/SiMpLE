# SiMpLE Minimal Configuration LiDAR odometry
Simple Mapping and Localisation Estimation ([SiMpLE](https://doi.org/10.1177/02783649241235325)) is a low-drift and low-configuration LiDAR odometry method providing accurate localization and mapping with a variety of sensors.

[Click here for a demo!](https://github.com/vb44/SiMpLE/assets/63623876/5fc7663b-f255-4b8c-a57b-f8288e56439e)

**The original paper code is available as a release [here](https://github.com/vb44/SiMpLE/releases/tag/v1.0.0).**

We perform among state-of-the-art LiDAR odometry methods with a simple and use case agnostic algorithm.
The method is derived from using raw point clouds. We do not modify or develop a new variant of any existing algorithm.
The code is written for readability and easy understanding with the methodology described in the paper.
The software can be optimised for increased performance.
Find our paper [here](https://doi.org/10.1177/02783649241235325).

We have tested the algorithm using LiDARs with various point cloud densities, fields of view, and scan patterns,
* Velodyne HDL-64 KITTI dataset 
* OS1-64 from the MulRan dataset
* Velodyne VLP-16 using a self-created dataset
* Livox Horizon using a self-created dataset

The motivation stems from reducing the complexity and configuration burden of localisation algorithms.

The code is portable, easy to understand, and modify. 

The following includes:
1. [A summary of the LiDAR odometry method](#method).
2. [The hardware requirements and dependencies installation](#hardware-and-dependencies).
3. [Installation of the SiMpLE repository](#installation).
4. [An example of using SiMpLE](#example).
5. [Sample results interpretation](#sample-results-interpretation).
6. [The effect of threading on real-time performance](#the-effect-of-threading-on-real-time-performance).
7. [Results on different operating systems and processors](#results-on-different-operating-systems-and-processors).
8. [Extensive Parameter List](#extensive-parameter-list)
9. [Other datasets](#other-datasets).
10. [References](#references).

## Method
The method has five configuration parameters.
1. ***rNew*** is the new point cloud spatial subsampling radius.
2. ***rMap*** is the spatial subsampling radius for the local map.
3. ***&sigma;*** is the standard deviation of the Gaussian reward function - see the paper for more details.
4. ***&varepsilon;*** is the convergence tolerance of the optimisation solver. 
5. ***rMin*** is optional and for computational benefit only. The default value is zero where it has no effect. Increasing this value eliminates the rings commonly formed around the sensor to reduce the size of the point cloud without losing geometric information.

There are also hardware settings:
* ***rMax*** is the maximum radius of the point cloud. This is hardware-dependent and not a configuration parameter.
 
## Using Docker
For easy reproducibility, a docker image has been provided. To use it, clone this repository and inside the repository directory run the following command to build the docker image:
```
docker build -t simple .
```

Inside another directory, acquire a dataset (this example will use the MulRan dataset) and create a configuration file called config.yaml containing the following (other dataset examples can be seen in the configs directory):
```
---
scanPath: ./dataset/MulRan/DCC_02/velodyne/
outputFileName: ./dataset/result
kitti: false

# Algorithm configuration.
sigma: 0.35
rMap: 2
rNew: 0.5
convergenceTol: 1e-3
minSensorRange: 10

# Hardware configuration.
maxSensorRange: 120
```

Then you can run the following command to execute simple:
```
docker run --volume ./:/dataset simple /dataset/config.yaml
```
The result output should be located in the same directory you ran this command from.

## Building from source
### Hardware and Dependencies
This implementation has been tested on Ubuntu 20.04.5/6 LTS (Focal Fossa) and Ubuntu 22.04.3 (Jammy Jellyfish) with an Intel Core i7-10700K CPU @ 3.80GHz x 16 and 62.5 GiB memory.
SiMpLE uses a few open-source libraries for reading the algorithm configuration file, Kd-Trees, matrix operations, optimization functions, and CPU threading.
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
* Install the *Dlib* library for the optimization solver [3, 4].
```bash
wget http://dlib.net/files/dlib-19.24.tar.bz2
tar xvf dlib-19.24.tar.bz2
cd dlib-19.24/
mkdir build && cd build
cmake ..
sudo make install
```
* Install the *yaml-cpp* library for reading the configuration file [5].
```bash
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp
mkdir build && cd build
cmake ..
sudo make install
```

Alternative options for any of the libraries can be used if desired.
The code is easy to change.

### Installation
Clone the repository.
```bash
git clone https://github.com/vb44/SiMpLE.git
```

Create a build folder in the repository.
```bash
cd SiMpLE/simple
mkdir build && cd build
```

Run CMake.
```bash
cmake ../
```

Make the executable.
```bash
make
```

### Example
<!-- Show example usage. -->
The code only works with *.bin* files in the KITTI format.
However, the code is very easy to modify to suit the desired inputs and outputs.
When compiled, the SiMpLE algorithm is run using a *.yaml* algorithm configuration file as shown below.
```bash
./simple config.yaml
```
Sample config files are included in the *config* folder.
An example is shown below.\
***Note that the --kitti argument is required to correct the scans for the KITTI dataset.*** 

```yaml
---
# Test setup.
scanPath: /path_to_scans/velodyne/
outputFileName: /path_to_output_files/fileName
kitti: true

# Algorithm configuration.
sigma: 0.3
rMap: 0.5
rNew: 0.3
convergenceTol: 1e-6
minSensorRange: 0

# Hardware configuration.
maxSensorRange: 120
```

## Sample Results and Evaluation
Sample results from the paper are available in the ***sampleResults*** folder for all reported datasets.\
The estimated trajectories can be evaluated in the *devkit* folder, which is a reduced version of the devkit provided by KITTI.

It is important to note that the KITTI estimates must be transformed to the ground truth frame using the sensor registration before evaluating the results. An example is provided in the ***sampleResults*** folder.

## Results on Different Operating Systems and Processors
The implementation allows for deterministic results for a varying number of threads on a given platform.
However, slight variations were observed when executing the algorithm on different machines.
Upon investigation, this was found to be caused by the Dlib optimisation library having slight numerical precision differences when searching for the best pose hypothesis.
Due to the nature of recursive pose estimation, the differences propagate and result in slight variations in the output trajectory.

Example results from the KITTI dataset executed on different operating systems and processors are displayed below. 
* Ubuntu 20.04.5: Desktop with Intel Core i7 (10th gen, 16 cores).
* Ubuntu 20.04.6: Laptop with Intel Core i7 (8 cores).
* Ubuntu 22.04.3: Desktop virtual machine with Intel Core i7 (10th gen, 10 cores).
* Windows 11: Desktop with Intel Pentium (8 cores).
* macOS Sonoma 14.2.1: MacBook Air with M1 core.

The slight differences in the KITTI results on different machines are shown below.

| **Sequence** | **Ubuntu 20.04.5** | **Ubuntu 20.04.6** | **Ubuntu 22.04.3** |   **Windows 11**   |   **macOS 14.2.1** |
|--------------|--------------------|--------------------|--------------------|--------------------|--------------------|
| **00**       |       0.6667       |       0.6585       |       0.6599       |       0.6570       |       0.6606       |
| **01**       |       0.7749       |       0.7750       |       0.7682       |       0.7786       |       0.7740       |
| **02**       |       0.6263       |       0.6452       |       0.6290       |       0.6298       |       0.6208       |
| **03**       |       0.7369       |       0.7378       |       0.7450       |       0.7507       |       0.7443       |
| **04**       |       0.4067       |       0.4063       |       0.4043       |       0.4056       |       0.4052       |
| **05**       |       0.3372       |       0.3298       |       0.3328       |       0.3213       |       0.3362       |
| **06**       |       0.2613       |       0.2607       |       0.2621       |       0.2595       |       0.2599       |
| **07**       |       0.4706       |       0.4768       |       0.4488       |       0.4622       |       0.4475       |
| **08**       |       0.8236       |       0.8238       |       0.8215       |       0.8233       |       0.8290       |
| **09**       |       0.5684       |       0.5420       |       0.5588       |       0.5476       |       0.5656       |
| **10**       |       0.6454       |       0.6318       |       0.6622       |       0.6321       |       0.6186       |
| **Average**  |     **0.5744**     |     **0.5716**     |     **0.5720**     |     **0.5698**     |     **0.5693**     |


## Extensive Parameter List
The five parameters listed at the beginning of the page, namely, *rNew*, *rMap*, *&sigma;*, *&varepsilon;*, and *rMin*,  are the algorithm configuration parameters.
The use of opensource libraries such as nanoflann and Dlib introduce additional parameters into the implementation.
We use the default parameters often hardcoded in the libraries and do not change them.
Hence, these parameters are not included in the algorithm configuration list.
SiMpLE has been extensively tested on numerous benchmark datasets in different environments and LiDARs with different characteristics, and we have not needed to change any default parameters.
For complete transparency, an extensive list of all identifiable parameters and their source is displayed below.

### Algorithm Configuration Parameters
| Parameter    |          Source           |                            Use                            |
|--------------|:-------------------------:|:---------------------------------------------------------:|
| rNew         | Algorithm configuration.  | New scan subsampling.                                     |
| rMap         | Algorithm configuration.  | Local map subsampling.                                    |
| &sigma;      | Algorithm configuration.  | Reward standard deviation.                                |
| &varepsilon; | Algorithm configuration.  | Optimisation exit condition.                              |
| rMin         | Algorithm configuration.  | Minimum sensor range used for reducing point cloud size.  |

### Hardware Parameters
| Parameter |         Source          |             Use             |
|-----------|:-----------------------:|:---------------------------:|
| rMax      | Hardware specification. | Maintaining local map size. |


### Default Unchanged Parameters
 Other unchanged, default parameters that are introduced by opensource libraries as they appear in the code. These parameters have not been changed for all experiments,

| Parameter                  |                            Source                            |     Value     |                                                                                                                                                     Use                                                                                                                                                    |
|----------------------------|:------------------------------------------------------------:|:-------------:|:----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:|
| leafSize                   | KdTree library (nanoflann)                                   | 10            | Used to construct the KD-tree. Default recommended value used.                                                                                                                                                                                                                                             |
| derivativeEps              | Dlib (API call to find ‘min using approximate derivatives’)  | 1e-7          | Derivative step. Unchanged from the library.                                                                                                                                                                                                                                                               |
| wolfeRho                   | Dlib (API call to ‘bfgs search strategy’)                    | 0.01          | Set as a const parameter in the library (Nocedal and Wright (1999)).                                                                                                                                                                                                                                       |
| wolfeSigma                 | Dlib (API call to ‘bfgs search strategy’)                    | 0.9           | Set as a const parameter in the library (Nocedal and Wright (1999)).                                                                                                                                                                                                                                       |
| Line search, maxIterations | Dlib (API call to ‘bfgs search strategy’)                    | 100           | Set as a const parameter in the library.                                                                                                                                                                                                                                                    

## Updates
* 21/04/2025: Refactor project to improve style and timeliness. Improving for future integration.
* 05/04/2024: Refactor project for better readability and execution time improvement.

## References
[1] [eigen library](https://eigen.tuxfamily.org/dox/GettingStarted.html) \
[2] [nanoflann library](https://github.com/jlblancoc/nanoflann) \
[3] [Dlib library](http://dlib.net/compile.html) \
[4] [Dlib C++ install for CMake](https://learnopencv.com/install-dlib-on-ubuntu/)\
[5] [yaml-cpp library](https://github.com/jbeder/yaml-cpp)
