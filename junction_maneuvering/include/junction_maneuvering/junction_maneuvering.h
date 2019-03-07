#ifndef JUNCTION_MANEUVERING_H
#define JUNCTION_MANEUVERING_H

#include <vector>
#include <cmath>
#include <junction_maneuvering/structs.h>

class JunctionManeuvering
{

public:
    JunctionManeuvering();
    ~JunctionManeuvering();
    
    // setter methods
    bool setGoal(int goal, double turn_direction, double distance, Gateways detected_gateways);
    void setParams(double laser_robot_center_offset_x);
    
    // getter methods for getting orientation at different states in state machine
    double getTurnAngle();
    double getPassingOrientation();

    // determine & get current state
    bool isStateChanged(double monitored_distance, double monitored_heading, Gateways detected_gateways);
    int getState();

    // compute orienations & distances
    bool computePassingOrientation(Gateways detected_gateways);

    // resets door passing
    void reset();

private:
    // door direction: 0-left|1-front|2-left
    int goal_;
    
    // current state
    int state_;

    // orientations
    double turn_angle_;
    double passing_orientation_;
    
    // distances
    double turn_range_;
    double distance_;
};

#endif
