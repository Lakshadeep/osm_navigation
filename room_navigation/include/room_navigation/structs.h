#ifndef ROOM_NAVIGATION_STRUCTS_H
#define ROOM_NAVIGATION_STRUCTS_H

#include <vector>
#include <cmath>
#include <string>

struct Point
{
    double x;
    double y;
};

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
    double front_range_x;
    double front_range_y;
};

struct Gateways
{
    Hallway hallway;
    TJunction t_junction;
    XJunction x_junction;
};

struct WallSide
{
    double radius;
    double angle;
    Point corner1;
    Point corner2;
};

struct DoorSide
{
    double radius;
    double angle;
    Point corner1;
    Point corner2;
};

struct Pillar
{
    double radius;
    double angle;
};

struct Feature
{
    std::string type;
    double height;
    double breast;
    double width;
    Point position;
};

struct SemanticFeatures
{
    std::vector<WallSide> wall_sides;
    std::vector<DoorSide> door_sides;
    std::vector<Pillar> pillars;
    std::vector<Feature> features;
};

#endif