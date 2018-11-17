#ifndef PHYSICS_2D_PHYSICS_UTIL_H
#define PHYSICS_2D_PHYSICS_UTIL_H

#include <unistd.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 2 Dimensional Vector
 *  [0] = X dimension
 *  [2] = Y Dimension
 */
typedef float EcsVector2D[2];

/**
 * Axis-Aligned Bounding Box
 *  [0] = Min X value
 *  [1] = Min Y value
 *  [2] = Max X value 
 *  [3] = Max Y value
 */
typedef float EcsAABB[4];

typedef float EcsMatrix3x3[3][3];
#define EcsMatrix3x3_Identity() {{1,0,0},{0,1,0},{0,0,0}}

/**
 * 
 * distance: Overlaping distant of the collision 
 * direction[2] Projection Vector
 */
typedef struct EcsCollisionInfo {
    float distance;
    EcsVector2D direction;
} EcsCollisionInfo;

/**
 * Components needed to represent a collider
 *  [0] : EcsVector2D
 *  [1] : EcsCircleCollider (Can be NULL)
 *  [2] : EcsPolygonCollider (Can be NULL)
 */
typedef void* EcsColliderData[3];
#define EcsColliderData_Vector2(pColliderData) ((EcsVector2D*)((*pColliderData)[0]))
#define EcsColliderData_Circle(pColliderData) ((EcsCircleCollider*)((*pColliderData)[1]))
#define EcsColliderData_Polygon(pColliderData) ((EcsPolygonCollider*)((*pColliderData)[2]))

int8_t EcsColliderData_getAABB(EcsColliderData *collider, EcsAABB *aabb_out);
int8_t EcsAABBTest(EcsAABB *a, EcsAABB *b);

float EcsVector2D_get_angle(EcsVector2D *vector);
float EcsVector2D_get_magnitude(EcsVector2D *vector);
int8_t EcsVector2D_get_normal(EcsVector2D *vector, EcsVector2D *vector_out);
int8_t EcsVector2D_normalize(EcsVector2D *vector, EcsVector2D *vector_out);
int8_t EcsVector2D_scale(EcsVector2D *vector, float scale, EcsVector2D *vector_out);
int8_t EcsVector2D_cross(EcsVector2D* vector_a, EcsVector2D *vector_b, EcsVector2D *vector_out);
int8_t EcsVector2D_add(EcsVector2D* vector_a, EcsVector2D *vector_b, EcsVector2D *vector_out);
int8_t EcsVector2D_sub(EcsVector2D* vector_a, EcsVector2D *vector_b, EcsVector2D *vector_out);

float EcsVector2D_dot(EcsVector2D* vector_a, EcsVector2D *vector_b);
float EcsVector2D_distance(EcsVector2D* vector_a, EcsVector2D *vector_b);
float EcsVector2D_distanceSqrt(EcsVector2D* vector_a, EcsVector2D *vector_b);
float EcsVector2D_angle(EcsVector2D* vector_a, EcsVector2D *vector_b);

void EcsMatrix3x3_add_rotation(EcsMatrix3x3 *matrix, float rad);
void EcsMatrix3x3_add_translation(EcsMatrix3x3 *matrix, EcsVector2D *translation);
int8_t EcsMatrix3x3_transform(EcsMatrix3x3 *matrix, EcsVector2D *src, EcsVector2D *dest, size_t size);

#ifdef __cplusplus
}
#endif

#endif

