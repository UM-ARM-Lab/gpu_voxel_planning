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
  * [unknown_graph_planner](https://github.com/UM-ARM-Lab/unknown_graph_planner)
  * [arm_pointcloud_utilities](https://github.com/UM-ARM-Lab/arm_pointcloud_utilities)


## Running live demo
In different terminals run
1. `roscore`
2. `rviz`. Load the `.rviz` config file here
3. Startup Real Victor
   1. `ssh realtime`
   2. `rcblizzard` (or source the appropriate computer for ros master), 
   3. `roslaunch gpu_voxel_models dual_arm_lcm_bridge.launch`. NOTE! This is not the standard `roslauch victor_hardware_interface dual_arm_lcm_bridge.launch`. Also note, `victor_hardware_interface/dual_arm_lcm_bridge.launch` may change. `master` of Jan 2020 works.
4. `roslaunch gpu_voxel_planning real_victor_setup.launch`
5. `rosrun gpu_voxel_planning live_demo`
