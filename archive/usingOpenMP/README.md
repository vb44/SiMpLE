# Using OpenMP for Threading
The following provides instructions for replacing the TBB threading with OpenMP.
OpenMP is documented to support Intel, AMD, and ARM compilers (https://www.openmp.org/resources/openmp-compilers-tools/) and can be used as an alternative to TBB if desired.

TBB is only used in the objective function evaluation.
Hence, to use OpenMP instead of TBB, the following changes are required to the src directory.
1. Replace the *objectiveFunction.cpp*, *objectiveFunction.h*, and *CMakeLists.txt* files from the src directory with the files in this folder.
2. Re-build and use as usual.

The objective function is changed to use OpenMP instead of TBB.
The CMakeLists file removes the TBB package dependency and adds the OpenMP dependency.