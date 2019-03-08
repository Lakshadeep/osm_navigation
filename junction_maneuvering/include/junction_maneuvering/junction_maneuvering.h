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
    bool setGoal(int goal, int turn_direction, double distance, Gateways detected_gateways);
    void setParams(double laser_robot_center_offset_x);
    
    // getter methods for getting orientation at different states in state machine
    bool computeInitialOrientation(Gateways detected_gateways);
    double getTurnAngle();
    double getPassingOrientation();
    double getInitialOrientation();

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
    double initial_orientation_;
    
    // distances
    double turn_range_;
    double turn_range_x_, turn_range_y_;

    double distance_;
    int turn_direction_;
};

#endif
