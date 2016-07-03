#ifndef ROTATION_H
#define ROTATION_H

#ifdef __cplusplus
extern "C" {
#endif


/**
 * arg angles should be the the nautical angles of the sub in radians, ordered
 * yaw, pitch, roll.
 * arg matrix is populated with the 3x3 rotation matrix to transform a linear
 * vector from world space to sub space. Multiply by its transpose to go from
 * sub space to world space.
 */
void rotation(float const angles[3], float matrix[3][3]);

/**
 * arg angles should be the the nautical angles of the sub in radians, ordered
 * yaw, pitch, roll, although yaw (angles[0]) is never actually accessed.
 * arg matrix is populated with the 3x3 rotation matrix to transform a vector
 * of angles yaw, pitch, and roll from world space to sub space.
 */
void rotation_angles(float const angles[3], float matrix[3][3]);


#ifdef __cplusplus
}
#endif

#endif
