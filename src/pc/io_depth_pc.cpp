#include <stdio.h>
#include <assert.h>

static FILE *depth_stream;

void io_depth_init(char const *path)
{
	depth_stream = fopen("depth_in", "r");
	return;
}

float io_depth()
{
	assert(depth_stream);
	float val;
	fscanf(depth_stream, "%f ", &val);
	return val;
}
