# gpu_voxel_planning

## Installation


1. Install [gpu-voxels](http://www.gpu-voxels.org/documentation/)

   * Install Cuda Drivers (Works with Cuda 9.1, has failed with Cuda 8.0 and Cuda 10.0)
   * Install [required libraries](http://www.gpu-voxels.org/documentation/prerequisites/)
   * Patch the GLM visualizer (still needed as of 01/01/18)
   * Follow the [Usage as a library](http://www.gpu-voxels.org/documentation/usage-as-a-library/) instructions and update CMAKE_PREFIX_PATH in your .bashrc file
    
    `export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:~/gvl`
   
2. Install armlab dependencies: 

  * arc_utilities, branch CleanUpDijkstras
  * graph_planner
