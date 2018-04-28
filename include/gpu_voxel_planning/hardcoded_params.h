#ifndef GPU_VOX_HARDCODED_PARAMS_H
#define GPU_VOX_HARDCODED_PARAMS_H


/* Number of iterations to run shortcut smoothing */
#define SMOOTHING_ITERATIONS 30

/* Expected num total voxels in world in collision */
#define UNEXPLORED_BIAS 0

/* Max planning time in seconds */
#define PLANNING_TIMEOUT 5

#define USE_KNOWN_OBSTACLES false

#define ALL_OBSTACLES_KNOWN false

#define NUM_STEPS_FOR_ADDING_COLLISION 5

#define PEG_IN_HOLE false

#define PLAN_ONLY false

#endif
