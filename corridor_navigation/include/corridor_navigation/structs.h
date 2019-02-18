#ifndef CORRIDOR_NAVIGATION_STRUCTS_H
#define CORRIDOR_NAVIGATION_STRUCTS_H

#include <vector>
#include <cmath>

struct Hallway
{
    double left_angle;
    double right_angle;
    double left_range;
    double right_range;
};

struct TJunction
{
    double left_turn_angle;
    double left_turn_range;
    double right_turn_angle;
    double right_turn_range;
    double front_angle;
    double front_range;
};

struct XJunction
{
    double left_turn_angle;
    double left_turn_range;
    double right_turn_angle;
    double right_turn_range;
    double front_angle;
    double front_range;
};

struct Gateways
{
    Hallway hallway;
    TJunction t_junction;
    XJunction x_junction;
};

// NOTE: currently doors are not considered as there is no distinction between doors on the left and right side

#endif