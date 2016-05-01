#include <stdio.h>
#include <assert.h>

#include "dbg.h"

static FILE *depth_stream;

void io_depth_init(char const *path)
{
	depth_stream = fopen("depth_in", "r");
	if (!depth_stream)
	{
		DEBUG("Failed to open %s", path);
	}
	return;
}

float io_depth()
{
	assert(depth_stream);
	float val;
	if(fscanf(depth_stream, "%f ", &val) != 1)
	{
		DEBUG("Failed reading depth");
		return 0.f;
	}
	return val;
}
