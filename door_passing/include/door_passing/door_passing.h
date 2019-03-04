#ifndef DOOR_PASSING_H
#define DOOR_PASSING_H

#include <vector>
#include <cmath>
#include <door_passing/structs.h>

class DoorPassing
{

public:
    DoorPassing();
    ~DoorPassing();
    
    // setter methods
    bool setGoal(int goal, double distance_inside, Gateways detected_gateways);
    void setParams(double laser_robot_center_offset_x);
    
    // getter methods for getting orientation at different states in state machine
    double getInitialOrientation();
    double getTurnAngle();
    double getPassingOrientation();
    double getInsideOrientation();

    // determine & get current state
    bool isStateChanged(double monitored_distance, double monitored_heading, Gateways detected_gateways);
    int getState();

    // compute orienations & distances
    bool computeInitialOrientation(int goal, Gateways detected_gateways);
    bool computePassingOrientation(Gateways detected_gateways);
    bool computeInsideOrientation(Gateways detected_gateways);
    bool computeDistanceToDoor(Gateways detected_gateways);

    // resets door passing
    void reset();

private:
    // door direction: 0-left|1-front|2-left
    int goal_;
    
    // current state
    int state_;

    // params
    double laser_robot_center_offset_x_;

    // orientations
    double intial_orientation_;
    double turn_angle_;
    double passing_orientation_;
    double orientation_inside_;
    
    // distances
    double turn_range_;
    double distance_to_door_;
    double distance_inside_;
};

#endif
