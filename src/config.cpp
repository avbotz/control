#include "config.hpp"

Config config = { {}, {
4, 0, 0,
4, 0, 0,
4, .002, 300,
4, .0005, 600,
4, .0005, 600,
4, .0000, 600,
//forward, strafe, depth, yaw, pitch, roll
0, 0, 1, 0, -1, -1,
0, 0, -1, 0, 1, -1,
0, 0, -1, 0, -1, 1,
0, 0, 1, 0, 1, 1,
1, 1, 0, 1, 0, 0,
-1, 1, 0, 1, 0, 0,
-1, 1, 0, -1, 0, 0,
1, 1, 0, -1, 0, 0}};
