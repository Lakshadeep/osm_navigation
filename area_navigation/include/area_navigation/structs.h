#ifndef AREA_NAVIGATION_STRUCTS_H
#define AREA_NAVIGATION_STRUCTS_H

#include <vector>
#include <cmath>
#include <string>

struct Point
{
    double x;
    double y;
    double z;
};

struct Euler
{
    double roll;
    double pitch;
    double yaw;
};

struct NavigationSign
{
    int direction;  // 0 - Left | 2 - right
    int type;       // 0 - exit
    Point position;
    Euler orientation;
};

#endif