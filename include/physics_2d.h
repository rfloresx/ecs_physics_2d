#ifndef PHYSICS_2D_H
#define PHYSICS_2D_H

#include <unistd.h>

#include "physics_util.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef EcsVector2D EcsPoint;

typedef struct EcsCircleCollider {
    float radius;
} EcsCircleCollider;

typedef struct EcsPolygonCollider {
    EcsPoint *points;
    int8_t points_count; //MAX 128
} EcsPolygonCollider;

int8_t EcsPhysis2dCollisionCheck(
    EcsColliderData *collider_a, 
    EcsColliderData *collider_b, 
    EcsCollisionInfo *collision_out);


#ifdef __cplusplus
}
#endif

#endif

