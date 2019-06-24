#ifndef GPU_VOX_HARDCODED_PARAMS_H
#define GPU_VOX_HARDCODED_PARAMS_H


/* Number of iterations to run shortcut smoothing */
#define SMOOTHING_ITERATIONS 100

/* Expected num total voxels in world in collision */
#define UNEXPLORED_BIAS 0.0

/* Max planning time in seconds */
#define PLANNING_TIMEOUT 30

#define USE_KNOWN_OBSTACLES true

#define MAKE_TABLE true

#define MAKE_SLOTTED_WALL false

#define ALL_OBSTACLES_KNOWN false

#define NUM_STEPS_FOR_ADDING_COLLISION 5

#define DISTANCE_FOR_ADDING_CHS 0.05

#define TABLE_WORLD true

#define PEG_IN_HOLE false

#define PLAN_ONLY true

#define DO_CONTROL true

#define DO_PLAN true

#define VOX_CONTROLLER_THRESHOLD 50

#define VIDEO_VISUALIZE true

#define EXECUTION_DELAY_us 50000

#define REAL_ROBOT false

#define ALLOWED_KNOWN_OBSTACLES 0

#define DO_RANDOM_WIGGLE false

#define DO_IOU_WIGGLE false

#define DO_PLANNING_FIRST true

#endif
