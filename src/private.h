#ifndef __PRIVATE_H__
#define __PRIVATE_H__

#include "include/physics_2d.h"

#ifndef false 
#define false 0
#endif 
#ifndef true 
#define true 1
#endif

#define ERR -1

#define VECTOR_X(vector) ((*vector)[0])
#define VECTOR_Y(vector) ((*vector)[1])

// EcsAABB[0] =
// EcsAABB
// EcsAABB
// EcsAABB

#define AABB_MIN_X(aabb) ((*aabb)[0])
#define AABB_MIN_Y(aabb) ((*aabb)[1])
#define AABB_MAX_X(aabb) ((*aabb)[2])
#define AABB_MAX_Y(aabb) ((*aabb)[3])

// EcsColliderData[0] = Position, 
// EcsColliderData[1] = CircleCollider, 
// EcsColliderData[2] = PolygonCollider
#define POSITION 0
#define CIRCLE 1
#define POLYGON 2

#define GET_POSITION(collider) ((EcsPoint*)((*collider)[POSITION]))
#define GET_CIRCLE(collider)   ((EcsCircleCollider*)((*collider)[CIRCLE]))
#define GET_POLYGON(collider)  ((EcsPolygonCollider*)((*collider)[POLYGON]))

#define COLLIDER_DATA(collider) ((ColliderData_t*)collider)
#define GET_COLLIDER_TYPE(collider) ((GET_CIRCLE(collider) != NULL) ? CIRCLE :\
                                 (GET_POLYGON(collider) != NULL) ? POLYGON : ERR)

#define POLYGON_COLLIDER_START(polygon) (polygon->points)
#define POLYGON_COLLIDER_END(polygon) (&(polygon->points[polygon->points_count]))

typedef struct ColliderData {
    EcsPoint *position;
    EcsCircleCollider *circle;
    EcsPolygonCollider *polygon;
} ColliderData_t;


#endif