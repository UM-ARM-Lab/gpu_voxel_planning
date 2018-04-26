#ifndef GPU_VOX_HARDCODED_PARAMS_H
#define GPU_VOX_HARDCODED_PARAMS_H


/* Number of iterations to run shortcut smoothing */
#define SMOOTHING_ITERATIONS 30

/* Expected num total voxels in world in collision */
#define UNEXPLORED_BIAS 100

/* Max planning time in seconds */
#define PLANNING_TIMEOUT 5

#define USE_KNOWN_OBSTACLES true

#define NUM_STEPS_FOR_ADDING_COLLISION 5

#endif
