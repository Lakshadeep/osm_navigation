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
    bool setGoal(int goal, double distance_inside, Gateways detected_gateways);

    void setTurnAngle(double turn_angle);
    void setTurnRange(double turn_range);
    
    double getTurnAngle();
    double getTurnRange();
    double getPassingOrientation();
    double getInsideOrientation();

    bool isStateChanged(double monitored_distance, double monitored_heading, Gateways detected_gateways);

    int getState();

    bool computeInitialOrientation(int goal, Gateways detected_gateways);
    bool isInitialOrientationCorrect(double monitored_heading);
    double getInitialOrientation();

    bool computePassingOrientation(Gateways detected_gateways);
    bool computeDistanceToDoor(Gateways detected_gateways);

    bool computeInsideOrientation(Gateways detected_gateways);

    void reset();
    
private:
    int goal_;
    int state_;

    double intial_orientation_;

    double turn_range_;
    double turn_angle_;
    
    double distance_to_door_;
    double passing_orientation_;

    double distance_inside_;
    double orientation_inside_;
};

#endif
