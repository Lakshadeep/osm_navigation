#ifndef SEMANTIC_MAP_H
#define SEMANTIC_MAP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************
 Structs
**************************************************************************/

typedef struct
{
  float x;
  float y;
} point_t;

typedef struct
{
  point_t corner1;
  point_t corner2;
} wall_side_t;

typedef struct
{
  point_t corner1;
  point_t corner2;
} door_side_t;

typedef struct
{
  point_t position;
} feature_t;

typedef struct
{
  int no_of_corners;
  point_t *corners;
  point_t point;
} pillar_t;


// Description for a semantic map
typedef struct
{
  float max_x, max_y, min_x, min_y;
  int no_of_wall_sides, no_of_door_sides, no_of_pillars, no_of_features;
  wall_side_t *wall_sides;
  door_side_t *door_sides;
  pillar_t *pillars;
  feature_t *features;  
} semantic_map_t;


/**************************************************************************
 Functions
**************************************************************************/

// Create a new (empty) map
semantic_map_t *semantic_map_alloc(void);

// Destroy a map
void semantic_map_free(semantic_map_t *map);

#ifdef __cplusplus
}
#endif

#endif
