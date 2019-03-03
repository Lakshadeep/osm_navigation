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
    bool setGoal(int goal, Gateways detected_gateways);

    void setTurnAngle(double turn_angle);
    void setTurnRange(double turn_range);
    
    double getTurnAngle();
    double getTurnRange();

    bool isStateChanged(double monitored_distance, double monitored_heading, Gateways detected_gateways);

    int getState();

    bool computeInitialOrientation(int goal, Gateways detected_gateways);
    bool isInitialOrientationCorrect(double monitored_heading);
    double getInitialOrientation();
    
private:
    double intial_orientation_;
    double turn_angle_;
    double turn_range_;
    double distance_to_door_;
    double distance_after_passing_;
    int state_;
};

#endif
