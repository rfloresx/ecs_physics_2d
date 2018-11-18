#include <SDL2/SDL.h>
#include <OpenGL/gl.h>
#include <physics_2d/physics_2d.h>

typedef uint8_t Color[3];

void DrawPolygon(SDL_Renderer *display, EcsColliderData *polygonCollider, Color* color) {
    EcsPoint buffer[128];
    SDL_Point points[129];
    EcsMatrix3x3 transform = EcsMatrix3x3_Identity();
    EcsPolygonCollider *polygon = EcsColliderData_Polygon(polygonCollider);
    EcsMatrix3x3_add_translation(&transform, EcsColliderData_Vector2(polygonCollider));
    EcsMatrix3x3_transform(&transform, polygon->points, buffer, polygon->points_count);
    
    uint8_t i;

    for (i = 0; i < polygon->points_count; i++) {
        points[i].x = (int)(buffer[i][0]);
        points[i].y = (int)(buffer[i][1]);
    }
    
    points[i].x = (int)(buffer[0][0]);
    points[i++].y = (int)(buffer[0][1]);

    SDL_SetRenderDrawColor(display, (*color)[0], (*color)[1], (*color)[2], SDL_ALPHA_OPAQUE);
    SDL_RenderDrawLines(display, points, i);   
}

void DrawCircle(SDL_Renderer *display, EcsColliderData *circleCollider, Color* color) {
    #define STEPS 50
    EcsPoint buffer[STEPS];
    SDL_Point points[129];
    EcsMatrix3x3 transform = EcsMatrix3x3_Identity();
    uint8_t i;
    EcsMatrix3x3_add_rotation(&transform, (2.0*M_PI)/STEPS);
    buffer[0][0] = EcsColliderData_Circle(circleCollider)->radius;
    buffer[0][1] = 0;
    for (i = 1; i < STEPS; i++) {
        EcsMatrix3x3_transform(&transform, &buffer[i-1], &buffer[i], 1);
    }
    EcsVector2D *pos = EcsColliderData_Vector2(circleCollider);
    for (i = 0; i < STEPS; i++) {
        points[i].x = (int)(buffer[i][0] + (*pos)[0]);
        points[i].y = (int)(buffer[i][1] + (*pos)[1]);
    }
    
    points[i].x = (int)(buffer[0][0] + (*pos)[0]);
    points[i++].y = (int)(buffer[0][1] + (*pos)[1]);

    SDL_SetRenderDrawColor(display, (*color)[0], (*color)[1], (*color)[2], SDL_ALPHA_OPAQUE);
    SDL_RenderDrawLines(display, points, i);   
}

void DrawCollider(SDL_Renderer *display, EcsColliderData *colliderData, Color *color) {
    if (EcsColliderData_Circle(colliderData) != NULL) {
        DrawCircle(display, colliderData, color);
    }
    if (EcsColliderData_Polygon(colliderData) != NULL) {
        DrawPolygon(display, colliderData, color);
    }
}

void DrawCollisionInfo(SDL_Renderer *display, EcsVector2D *pos, EcsCollisionInfo *info, Color *color) {
    EcsVector2D end;
    EcsVector2D_scale(&(info->direction), info->distance, &end);
    EcsVector2D_add(pos, &end, &end);
    
    SDL_SetRenderDrawColor(display, (*color)[0], (*color)[1], (*color)[2], SDL_ALPHA_OPAQUE);
    SDL_RenderDrawLine(display, (*pos)[0], (*pos)[1], end[0], end[1]);
}

typedef struct Circle {
    EcsVector2D position;
    EcsCircleCollider collider;
    EcsColliderData data;
} Circle;
#define DEFINE_CIRCLE(name, r) Circle name = {{0,0},{r},{&name.position, &name.collider, NULL}}

typedef struct Polygon {
    EcsVector2D position;
    EcsPolygonCollider collider;
    EcsColliderData data;
    EcsPoint points[4];
}Polygon;
#define POS(x,y) {x,y}
#define DEFINE_POLYGON(name, c, p0, p1, p2, p3) Polygon name = {{0,0},{name.points, c}, {&name.position, NULL, &name.collider}, {p0,p1,p2,p3}}



Color cWhite = {255,255,255};
Color cGray  = {100,100,100};
Color cRed   = {255,000,000};
Color cGreen = {000,255,000};
Color cBlue  = {000,000,255};

DEFINE_CIRCLE(circle1, 50);
DEFINE_CIRCLE(circle2, 50);
DEFINE_CIRCLE(circle3, 50);
DEFINE_POLYGON(polygon1, 4, POS(-50, -50), POS(50, -50), POS(50, 50), POS(-50, 50));
DEFINE_POLYGON(polygon2, 4, POS(-50, -50), POS(50, -50), POS(50, 50), POS(-50, 50));
DEFINE_POLYGON(polygon3, 4, POS(-50, -50), POS(50, -50), POS(50, 50), POS(-50, 50));
DEFINE_POLYGON(polygon4, 3, POS(0, -50), POS(50, 50), POS(-50, 50), POS(0,0));

#define COLLIDERS_COUNT 7
EcsColliderData* colliders[] = {
    &circle1.data,
    &circle2.data,
    &circle3.data,
    &polygon1.data,
    &polygon2.data,
    &polygon3.data,
    &polygon4.data,
    NULL
};
EcsCollisionInfo collisions[] = {
    {-1, {0,0}},
    {-1, {0,0}},
    {-1, {0,0}},
    {-1, {0,0}},
    {-1, {0,0}},
    {-1, {0,0}},
    {-1, {0,0}},
    {-1, {0,0}},
};

void MoveCollider(EcsColliderData *collider, EcsVector2D *mot) {
    EcsVector2D *pos = EcsColliderData_Vector2(collider);
    EcsVector2D_add(pos, mot, pos);
}

int main(int argc, char const *argv[])
{    
    SDL_Window *window;
    printf("SDL_Init: %d\n", SDL_Init(SDL_INIT_VIDEO));

    // Create an application window with the following settings:
    window = SDL_CreateWindow(
        "An SDL2 window",                  // window title
        SDL_WINDOWPOS_UNDEFINED,           // initial x position
        SDL_WINDOWPOS_UNDEFINED,           // initial y position
        640,                               // width, in pixels
        480,                               // height, in pixels
        SDL_WINDOW_OPENGL                 // flags - see below
    );

    // Check that the window was successfully created
    if (window == NULL) {
        // In the case that the window could not be made...
        printf("Could not create window: %s\n", SDL_GetError());
        return 1;
    }

    SDL_Renderer *display = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (display == NULL) {
        printf("Could not create Renderer: %s\n", SDL_GetError());
        goto EXIT;
    }
    
    int active = 0;
    EcsVector2D position = {0,0};
    SDL_Event e;
    do {
        SDL_SetRenderDrawColor(display, 0, 0, 0, SDL_ALPHA_OPAQUE);
        SDL_RenderClear(display);
        EcsCollisionInfo *end = collisions;
        EcsColliderData **iter = colliders;
        while(*iter != NULL) {
            Color *color = &cWhite;
            if (*iter == colliders[active]) {
                color = &cGreen;
            } else {
                if (EcsPhysis2dCollisionCheck(*iter, colliders[active], end)) {
                    position[0] = end->direction[0];
                    position[1] = end->direction[1];
                    EcsVector2D_scale(&position, end->distance, &position);
                    MoveCollider(*iter, &position);
                    end++;
                }
            }
            DrawCollider(display, *iter, color);
            iter++;
        }
        EcsCollisionInfo *infoIter = collisions;
        while (infoIter < end) {
            DrawCollisionInfo(display, EcsColliderData_Vector2(colliders[active]), infoIter, &cRed);
            infoIter++;
        }
        SDL_RenderPresent(display);

        SDL_PollEvent(&e);
        if (e.type == SDL_KEYDOWN) {
            position[0] = 0;
            position[1] = 0;
            
            if (e.key.keysym.scancode == SDL_SCANCODE_UP) {
                position[1] -= .1;
            }
            if (e.key.keysym.scancode == SDL_SCANCODE_DOWN) {
                position[1] += .1;
            }
            if (e.key.keysym.scancode == SDL_SCANCODE_LEFT) {
                position[0] -= .1;
            }
            if (e.key.keysym.scancode == SDL_SCANCODE_RIGHT) {
                position[0] += .1;
            }
            if (e.key.keysym.scancode == SDL_SCANCODE_N) {
                active = (active+1)%COLLIDERS_COUNT;
            }
            MoveCollider(colliders[active], &position);
        }
    } while(e.type != SDL_QUIT);

EXIT:
    SDL_DestroyWindow(window);

    // Clean up
    SDL_Quit();

    /* code */
    return 0;
}