#include "include/physics_2d.h"
#include "include/physics_util.h"
#include "private.h"
#include <math.h>

#define AXIS_MIN(minMax) ((minMax)[0])
#define AXIS_MAX(minMax) ((minMax)[1])

static int8_t EcsPhysis2dCollisionCheckCircleCircle(
    ColliderData_t *circle_a, 
    ColliderData_t *circle_b, 
    EcsCollisionInfo *collision_out);
static int8_t EcsPhysis2dCollisionCheckPolygonSat(
    ColliderData_t *polygon_a, 
    ColliderData_t *polygon_b, 
    EcsCollisionInfo *collision_out);
static int8_t EcsPhysis2dCollisionCheckCirclePolygonSat(
    ColliderData_t *circle, 
    ColliderData_t *polygon, 
    int8_t invert,
    EcsCollisionInfo *collision_out);
static float EcsPhysis2dCollisionCheckPolygonSatAxis(
    EcsPoint *vertices_a, int8_t size_a, 
    EcsPoint *vertices_b, int8_t size_b, 
    EcsCollisionInfo *collision_out, int8_t invert);
static void EcsPhysis2dCollisionCheckAxisSat(
    EcsVector2D *axis, 
    EcsVector2D *minMaxA,
    EcsVector2D *minMaxB,
    EcsCollisionInfo *collision_out);
static void EcsPhysis2d_getProjection(
    EcsVector2D *axis, 
    EcsPoint *vertices, int8_t size, 
    EcsVector2D *out);
static void EcsPhysis2d_getProjectionCircle(
    EcsVector2D *axis,
    EcsVector2D *center, float radius,
    EcsVector2D *out);
static void EcsPhysis2d_getClosestPoint(
    EcsVector2D *position, 
    EcsPoint *vertices, int8_t size,
    EcsVector2D *out
);

int8_t EcsPhysis2dCollisionCheck(
    EcsColliderData *collider_a, 
    EcsColliderData *collider_b, 
    EcsCollisionInfo *collision_out)
{
    if (collider_a == NULL || collider_b == NULL || GET_POSITION(collider_a) == NULL || GET_POSITION(collider_b) == NULL) {
        return false;
    }
    
    int8_t type_a = GET_COLLIDER_TYPE(collider_a);
    int8_t type_b = GET_COLLIDER_TYPE(collider_b);
    
    if (type_a == ERR || type_b == ERR) {
        return false;
    } else if (type_a == type_b) {
        if (type_a == CIRCLE) {
            return EcsPhysis2dCollisionCheckCircleCircle((ColliderData_t*)collider_a, (ColliderData_t*)collider_b, collision_out);
        } else if (type_b == POLYGON) {
            return EcsPhysis2dCollisionCheckPolygonSat((ColliderData_t*)collider_a, (ColliderData_t*)collider_b, collision_out);
        }
    } else {
        if (type_a == CIRCLE) {
            return EcsPhysis2dCollisionCheckCirclePolygonSat((ColliderData_t*)collider_a, (ColliderData_t*)collider_b, true, collision_out);
        } 
        return EcsPhysis2dCollisionCheckCirclePolygonSat((ColliderData_t*)collider_b, (ColliderData_t*)collider_a, false, collision_out);
    }
    return false;
}

static int8_t EcsPhysis2dCollisionCheckCircleCircle(
    ColliderData_t *circle_a, 
    ColliderData_t *circle_b, 
    EcsCollisionInfo *collision_out)
{   
    float totalRadius = circle_a->circle->radius + circle_b->circle->radius;
    float distSqrt = EcsVector2D_distanceSqrt(circle_a->position, circle_b->position);
    if (distSqrt > totalRadius*totalRadius) {
        return false;
    }
    EcsVector2D_sub(circle_b->position, circle_a->position, &(collision_out->direction));
    EcsVector2D_normalize(&(collision_out->direction), &(collision_out->direction));
    collision_out->distance = sqrtf(distSqrt)-totalRadius;
    return true;
}

static int8_t EcsPhysis2dCollisionCheckPolygonSat(
    ColliderData_t *polygon_a, 
    ColliderData_t *polygon_b, 
    EcsCollisionInfo *collision_out) 
{
    EcsPoint vertices_a[128];
    EcsPoint vertices_b[128];
    EcsMatrix3x3 transfor = {{1,0, VECTOR_X(polygon_a->position)}, {0,1, VECTOR_Y(polygon_a->position)}, {0,0,1}};
    
    EcsMatrix3x3_transform(&transfor, polygon_a->polygon->points, vertices_a, polygon_a->polygon->points_count);
    transfor[0][2] = VECTOR_X(polygon_b->position);
    transfor[1][2] = VECTOR_Y(polygon_b->position);
    EcsMatrix3x3_transform(&transfor, polygon_b->polygon->points, vertices_b, polygon_b->polygon->points_count);

    collision_out->distance = INFINITY;
    if (EcsPhysis2dCollisionCheckPolygonSatAxis(vertices_a, polygon_a->polygon->points_count,
                                                vertices_b, polygon_b->polygon->points_count, 
                                                collision_out, false) == INFINITY) {
        return false;
    }
    if (EcsPhysis2dCollisionCheckPolygonSatAxis(vertices_b, polygon_b->polygon->points_count,
                                                vertices_a, polygon_a->polygon->points_count, 
                                                collision_out, true) == INFINITY) {
        return false;
    }
    return true;
}

static int8_t EcsPhysis2dCollisionCheckCirclePolygonSat(
    ColliderData_t *circle, 
    ColliderData_t *polygon, 
    int8_t invert,
    EcsCollisionInfo *collision_out) 
{
    EcsPoint vertices_a[128];
    int8_t size_a = polygon->polygon->points_count;
    EcsMatrix3x3 transfor = {{1,0, VECTOR_X(polygon->position)}, {0,1, VECTOR_Y(polygon->position)}, {0,0,1}};
    EcsMatrix3x3_transform(&transfor, polygon->polygon->points, vertices_a, size_a); 

    EcsVector2D axis;
    EcsVector2D closestPoint;
    EcsVector2D minMaxA;
    EcsVector2D minMaxB;

    EcsPhysis2d_getClosestPoint(circle->position, vertices_a, size_a, &closestPoint);
    EcsVector2D_sub(circle->position, &closestPoint, &axis);
    EcsVector2D_normalize(&axis, &axis);

    EcsPhysis2d_getProjection(&axis, vertices_a, size_a, &minMaxA);
    EcsPhysis2d_getProjectionCircle(&axis, circle->position, circle->circle->radius, &minMaxB);

    //max0 < min1 || max1 < min0 
    if (AXIS_MAX(minMaxA) < AXIS_MIN(minMaxB) || AXIS_MAX(minMaxB) < AXIS_MIN(minMaxA)) {
        return false;
    }
    collision_out->distance = INFINITY;
    EcsPhysis2dCollisionCheckAxisSat(&axis, &minMaxA, &minMaxB, collision_out);

    for (int8_t i = 0; i < size_a; i++){
        EcsVector2D_sub(&vertices_a[(i+1) < size_a ? (i+1) : 0], &vertices_a[i], &axis);
        EcsVector2D_get_normal(&axis, &axis);
        EcsVector2D_normalize(&axis, &axis);
        
        EcsPhysis2d_getProjection(&axis, vertices_a, size_a, &minMaxA);
        EcsPhysis2d_getProjectionCircle(&axis, circle->position, circle->circle->radius, &minMaxB);
    
        //max0 < min1 || max1 < min0 
        if (AXIS_MAX(minMaxA) < AXIS_MIN(minMaxB) || AXIS_MAX(minMaxB) < AXIS_MIN(minMaxA)) {
            return false;
        }

        EcsPhysis2dCollisionCheckAxisSat(&axis, &minMaxA, &minMaxB, collision_out);
    }

    if (invert) {
        EcsVector2D_scale(&(collision_out->direction), -1, &(collision_out->direction));
    }

    return true;
}

static float EcsPhysis2dCollisionCheckPolygonSatAxis(
    EcsPoint *vertices_a, int8_t size_a, 
    EcsPoint *vertices_b, int8_t size_b, 
    EcsCollisionInfo *collision_out, int8_t invert)
{
    EcsVector2D axis;
    EcsVector2D minMaxA;
    EcsVector2D minMaxB;

    for (int8_t i = 0; i < size_a; i++){
        EcsVector2D_sub(&vertices_a[(i+1) < size_a ? (i+1) : 0], &vertices_a[i], &axis);
        EcsVector2D_get_normal(&axis, &axis);
        EcsVector2D_normalize(&axis, &axis);
        if (invert) {
            EcsPhysis2d_getProjection(&axis, vertices_a, size_a, &minMaxB);
            EcsPhysis2d_getProjection(&axis, vertices_b, size_b, &minMaxA);
        } else {
            EcsPhysis2d_getProjection(&axis, vertices_a, size_a, &minMaxA);
            EcsPhysis2d_getProjection(&axis, vertices_b, size_b, &minMaxB);
        }
        
        //max0 < min1 || max1 < min0 
        if (AXIS_MAX(minMaxA) < AXIS_MIN(minMaxB) || AXIS_MAX(minMaxB) < AXIS_MIN(minMaxA)) {
            return INFINITY;
        }

        EcsPhysis2dCollisionCheckAxisSat(&axis, &minMaxA, &minMaxB, collision_out);
    }
    return collision_out->distance;
}

static void EcsPhysis2dCollisionCheckAxisSat(
    EcsVector2D *axis, 
    EcsVector2D *minMaxA,
    EcsVector2D *minMaxB,
    EcsCollisionInfo *collision_out)
{
    float distmin;
    if (AXIS_MIN(*minMaxA) < AXIS_MIN(*minMaxB)) {
        distmin = AXIS_MIN(*minMaxB) - AXIS_MAX(*minMaxA);
        if (distmin < 0) {
            distmin = -distmin;
            EcsVector2D_scale(axis, -1, axis);
        }
    } else {
        distmin = AXIS_MIN(*minMaxA) - AXIS_MAX(*minMaxB);
        if (distmin < 0) {
            distmin = -distmin;
        } else {
            EcsVector2D_scale(axis, -1, axis);
        }
    }
    if (distmin <= collision_out->distance) {
        collision_out->distance = distmin;
        collision_out->direction[0] = (*axis)[0];
        collision_out->direction[1] = (*axis)[1];
    }
}

static void EcsPhysis2d_getProjection(
    EcsVector2D *axis, 
    EcsPoint *vertices, int8_t size, 
    EcsVector2D *out) 
{
    float min = EcsVector2D_dot(axis, &vertices[0]);
    float max = min;
    for (int i = 1; i < size; i++) {
        float t = EcsVector2D_dot(axis, &vertices[i]);
        if (t < min) {
            min = t;
        } 
        if (t > max) {
            max = t;
        }
    }
    VECTOR_X(out) = min;
    VECTOR_Y(out) = max;
}

static void EcsPhysis2d_getProjectionCircle(
    EcsVector2D *axis,
    EcsVector2D *center, float radius,
    EcsVector2D *out)
{
    float mid = EcsVector2D_dot(axis, center);
    VECTOR_X(out) = mid - radius;
    VECTOR_Y(out) = mid + radius;
}

static void EcsPhysis2d_getClosestPoint(
    EcsVector2D *position, 
    EcsPoint *vertices, int8_t size,
    EcsVector2D *out)
{
    float distance = MAXFLOAT;
    float currDist;

    for (int i = 0; i < size; i++) {
        currDist = EcsVector2D_distanceSqrt(position, &vertices[i]);
        if (currDist < distance) {
            distance = currDist;
            VECTOR_X(out) = VECTOR_X(&vertices[i]);
            VECTOR_Y(out) = VECTOR_Y(&vertices[i]);
        }
    }    
}