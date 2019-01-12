#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "semantic_localization/map/semantic_map.h"


// Create a new map
semantic_map_t *semantic_map_alloc(void)
{
  semantic_map_t *map;

  map = (semantic_map_t*) malloc(sizeof(semantic_map_t));

  map->min_x = 0;
  map->min_y = 0;
  map->max_x = 0;
  map->max_y = 0;

  map->no_of_wall_sides = 0;
  map->no_of_door_sides = 0;
  map->no_of_pillars = 0;
  map->no_of_features = 0;
  
  // Allocate storage for main map
  map->wall_sides = (wall_side_t*) NULL;
  map->door_sides = (door_side_t*) NULL;
  map->pillars = (pillar_t*) NULL;
  map->features = (feature_t*) NULL;
  
  return map;
}

// Destroy a map
void semantic_map_free(semantic_map_t *map)
{
  free(map->wall_sides);
  free(map->door_sides);
  free(map->pillars);
  free(map->features);
  free(map);
  return;
}

