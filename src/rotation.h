#ifndef ROTATION_H
#define ROTATION_H

#ifdef __cplusplus
extern "C" {
#endif


enum axis
{
	X,
	Y,
	Z
};

void rotation(float angles[3], float matrix[3][3]);


#ifdef __cplusplus
}
#endif

#endif
