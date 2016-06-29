#include <string.h>
#include <math.h>

#include "rotation.h"

/**
 * The ahrs outputs its orientation as nautical angles. These are intrinsic
 * angles of the form z-y'-x'' aka Heading, Elevation, and Bank. The rotations
 * are viewed as passive rotations because thrusters, accelerometers, et cetera
 * are relative to the sub itself, so we calculate the vector representing
 * down, north, or east relative to the sub.
 */
void rotation(float const angles[3], float matrix[3][3])
{
	float s1 = sinf(angles[0]), c1 = cosf(angles[0]);
	float s2 = sinf(angles[1]), c2 = cosf(angles[1]);
	float s3 = sinf(angles[2]), c3 = cosf(angles[2]);

	// The transposed rotation matrix for Tait-Bryan Z1-Y2-X3 angles
	memcpy(matrix, (float [3][3])
		{
			{c1*c2, c2*s1, -s2},
			{c1*s2*s3 - c3*s1, c1*c3 + s1*s2*s3, c2*s3},
			{s1*s3 + c1*c3*s2, c3*s1*s2 - c1*s3, c2*c3}
		}, 3 * sizeof(matrix[0]));

	return;
}

void rotation_angles(float const angles[3], float matrix[3][3])
{
	float s2 = sinf(angles[1]), c2 = cosf(angles[1]);
	float s3 = sinf(angles[2]), c3 = cosf(angles[2]);

	memcpy(matrix, (float [3][3])
		{
			{c2*c3, -s3, 0.f},
			{c2*s3, c3, 0.f},
			{-s2, 0.f, 1.f}
		}, sizeof(matrix[0]) * 3);

	return;
}
