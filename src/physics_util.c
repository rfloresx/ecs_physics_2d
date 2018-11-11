#include "include/physics_util.h"
#include "private.h"
#include <float.h>
#include <math.h>

static int8_t EcsColliderData_getCircleAABB(ColliderData_t *collider, EcsAABB *aabb_out) {
    AABB_MIN_X(aabb_out) = VECTOR_X(collider->position) - (collider->circle->radius);
    AABB_MIN_Y(aabb_out) = VECTOR_Y(collider->position) - (collider->circle->radius);
    AABB_MAX_X(aabb_out) = VECTOR_X(collider->position) + (collider->circle->radius);
    AABB_MAX_Y(aabb_out) = VECTOR_Y(collider->position) + (collider->circle->radius);
    return true;
}

static int8_t EcsColliderData_getPolygonAABB(ColliderData_t *collider, EcsAABB *aabb_out) {
    EcsPoint *iter = POLYGON_COLLIDER_START(collider->polygon);
    EcsPoint *end =  POLYGON_COLLIDER_END(collider->polygon);
    if (iter == NULL || iter >= end) {
        return false;
    }

    AABB_MIN_X(aabb_out) = FLT_MAX;
    AABB_MIN_Y(aabb_out) = FLT_MAX;
    AABB_MAX_X(aabb_out) = FLT_MIN;
    AABB_MAX_Y(aabb_out) = FLT_MIN;
    for (; iter < end; iter++) {
        if (AABB_MIN_X(aabb_out) > VECTOR_X(iter)) {
            AABB_MIN_X(aabb_out) = VECTOR_X(iter);
        }
        if (AABB_MIN_Y(aabb_out) > VECTOR_Y(iter)) {
            AABB_MIN_Y(aabb_out) = VECTOR_Y(iter);
        }
        if (AABB_MAX_X(aabb_out) < VECTOR_X(iter)) {
            AABB_MAX_X(aabb_out) = VECTOR_X(iter);
        }
        if (AABB_MAX_Y(aabb_out) < VECTOR_Y(iter)) {
            AABB_MAX_Y(aabb_out) = VECTOR_Y(iter);
        }
    }
    return true;
}

int8_t EcsColliderData_getAABB(EcsColliderData *collider, EcsAABB *aabb_out) 
{
    if (collider == NULL || GET_POSITION(collider) == NULL) {
        return false;
    }
    int8_t type = GET_COLLIDER_TYPE(collider);
    if (type == CIRCLE) {
        return EcsColliderData_getCircleAABB(COLLIDER_DATA(collider), aabb_out);
    } else if (type == POLYGON) {
        return EcsColliderData_getPolygonAABB(COLLIDER_DATA(collider), aabb_out);
    }
    return false;
}

int8_t EcsAABBTest(EcsAABB *a, EcsAABB *b) {
    if (a == NULL || b == NULL) {
        return false;
    }
    return !((AABB_MAX_X(a) < AABB_MIN_X(b)) || 
             (AABB_MAX_Y(a) < AABB_MIN_Y(b)) ||
             (AABB_MAX_X(b) < AABB_MIN_X(a)) || 
             (AABB_MAX_Y(b) < AABB_MIN_Y(a)));
}

float EcsVector2D_get_angle(EcsVector2D *vector) {
    if (vector == NULL) {
        return INFINITY;
    }
    float ang = atan2f(VECTOR_X(vector), VECTOR_Y(vector))*(180/M_PI);
    if (ang < 0) {
        ang += 360;
    }
    return ang;
}

float EcsVector2D_get_magnitude(EcsVector2D *vector) {
    if (vector == NULL) {
        return -1;
    }
    return sqrtf(VECTOR_X(vector)*VECTOR_X(vector)+VECTOR_Y(vector)*VECTOR_Y(vector));
}

int8_t EcsVector2D_get_normal(EcsVector2D *vector, EcsVector2D *vector_out) {
    if (vector == NULL || vector_out == NULL) {
        return false;
    }
    float x = -VECTOR_X(vector);
    VECTOR_X(vector_out) = VECTOR_Y(vector);
    VECTOR_Y(vector_out) = x;
    return true;
}

int8_t EcsVector2D_normalize(EcsVector2D *vector, EcsVector2D *vector_out) {
    float m = EcsVector2D_get_magnitude(vector);
    if (m <= 0) {
        return false;
    }
    VECTOR_X(vector_out) = VECTOR_X(vector)/m;
    VECTOR_Y(vector_out) = VECTOR_Y(vector)/m;
    return true;
}

int8_t EcsVector2D_scale(EcsVector2D *vector, float scale, EcsVector2D *vector_out) {
    if (vector == NULL || vector_out == NULL) {
        return false;
    }
    VECTOR_X(vector_out) = VECTOR_X(vector)*scale;
    VECTOR_Y(vector_out) = VECTOR_Y(vector)*scale;
    return true;
}

int8_t EcsVector2D_cross(EcsVector2D* vector_a, EcsVector2D *vector_b, EcsVector2D *vector_out) {
    if (vector_a == NULL || vector_b == NULL || vector_out == NULL) {
        return false;
    }
    float x = VECTOR_X(vector_b);
    VECTOR_X(vector_out) = VECTOR_X(vector_a) * VECTOR_Y(vector_b);
    VECTOR_Y(vector_out) = VECTOR_Y(vector_a) * x;
    return true;
}

int8_t EcsVector2D_add(EcsVector2D* vector_a, EcsVector2D *vector_b, EcsVector2D *vector_out) {
    if (vector_a == NULL || vector_b == NULL || vector_out == NULL) {
        return false;
    }
    VECTOR_X(vector_out) = VECTOR_X(vector_a) + VECTOR_X(vector_b);
    VECTOR_Y(vector_out) = VECTOR_X(vector_a) + VECTOR_Y(vector_b);
    return true;
}

int8_t EcsVector2D_sub(EcsVector2D* vector_a, EcsVector2D *vector_b, EcsVector2D *vector_out) {
    if (vector_a == NULL || vector_b == NULL || vector_out == NULL) {
        return false;
    }
    VECTOR_X(vector_out) = VECTOR_X(vector_a) - VECTOR_X(vector_b);
    VECTOR_Y(vector_out) = VECTOR_X(vector_a) - VECTOR_Y(vector_b);
    return true;
}

float EcsVector2D_dot(EcsVector2D* vector_a, EcsVector2D *vector_b) {
    if (vector_a == NULL || vector_b == NULL) {
        return INFINITY;
    }
    return VECTOR_X(vector_a) * VECTOR_X(vector_b) + VECTOR_Y(vector_a) * VECTOR_Y(vector_b);
}

float EcsVector2D_distance(EcsVector2D* vector_a, EcsVector2D *vector_b) {
    return sqrtf(EcsVector2D_distanceSqrt(vector_a, vector_b));
}

float EcsVector2D_distanceSqrt(EcsVector2D* vector_a, EcsVector2D *vector_b) {
    if (vector_a == NULL || vector_b == NULL) {
        return -1;
    }

    float x = VECTOR_X(vector_a) - VECTOR_X(vector_b);
    float y = VECTOR_Y(vector_a) - VECTOR_Y(vector_b);
    return  x * x + y * y;
}

float EcsVector2D_angle(EcsVector2D* vector_a, EcsVector2D *vector_b) {
    float angle = EcsVector2D_get_angle(vector_a) - EcsVector2D_get_angle(vector_b);

    return angle < 0 ? angle + 360 : angle;
}

#define MATRIX_GET(matrix, x, y) ((*matrix)[x][y])
int8_t EcsMatrix3x3_transform(EcsMatrix3x3 *matrix, EcsVector2D *src, EcsVector2D *dest, size_t size)
{
    if (matrix == NULL || src == NULL || dest == NULL) {
        return false;
    }
    for (size_t i = 0; i < size; i++) {
        float x = VECTOR_X(&src[i]);
        float y = VECTOR_Y(&src[i]);
        VECTOR_X(&dest[i]) = MATRIX_GET(matrix, 0, 0) * x +
                             MATRIX_GET(matrix, 0, 1) * y +
                             MATRIX_GET(matrix, 0, 2);
        VECTOR_Y(&dest[i]) = MATRIX_GET(matrix, 1, 0) * x +
                             MATRIX_GET(matrix, 1, 1) * y +
                             MATRIX_GET(matrix, 1, 2);
    }
    return true;
}