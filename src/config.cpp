#include "config.hpp"

//define things here
//aggregate-initalize master_config struct
//I think these are the config values luke meant but there are 69 values, not 66?
Config master_config = { 
	{} , 
	{ .25, .00042, 100,
	  .25, .00042, 100,
	  .001, .000017, 6,
	  .5, .006, 100,
	  .5, .006, 100,
	  .5, .006, 100,
	  -1, -1, 0, 1, 0, 0
	  -1, -1, 0, -1, 0, 0,
	   1, -1, 0, 1, 0, 0,
	   0, 0, -1, 0, 1, -1,
	   0, 0, -1, 0, -1, 1,
	   1, -1, 0, -1, 0, 0,
	   0, 0, 1, 0, -1, -1,
	   0, 0, 1, 0, 1, 1
	} 
};

