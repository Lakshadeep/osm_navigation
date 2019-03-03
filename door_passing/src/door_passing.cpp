#include <door_passing/door_passing.h>

DoorPassing::DoorPassing():state_(-1)
{
}

DoorPassing::~DoorPassing()
{
}

bool DoorPassing::setGoal(int goal, Gateways detected_gateways)
{
    if( goal == 0)
    {
        turn_range_ = detected_gateways.left_door.turn_range;
        turn_angle_ = detected_gateways.left_door.turn_angle;
    }
    else if (goal == 1)
    {
        turn_range_ = detected_gateways.front_door.turn_range;
        turn_angle_ = detected_gateways.front_door.turn_angle;
    }
    else if (goal == 2)
    {
        turn_range_ = detected_gateways.right_door.turn_range;
        turn_angle_ = detected_gateways.right_door.turn_angle;
    }
    else
    { 
        return false;
    }
    return true;
}

void DoorPassing::setTurnAngle(double turn_angle)
{
    turn_angle_ = turn_angle;
}
    
void DoorPassing::setTurnRange(double turn_range)
{
    turn_range_ = turn_range;
}
    
double DoorPassing::getTurnAngle()
{
    return turn_angle_;
}
    
double DoorPassing::getTurnRange()
{
    return turn_range_;
}

bool DoorPassing::isStateChanged(double monitored_distance, double monitored_heading, Gateways detected_gateways)
{
    if (state_ == -1 && monitored_distance > 0 && monitored_distance < turn_range_)
    {
        state_ == 0;
        return true;
    }
    else if (state_ == 0 && monitored_distance > turn_range_)
    {
        state_ = 1;
        return true;
    }
    else if (state_ == 1 && fabs(monitored_heading - turn_angle_) < 0.05)
    {
        state_ = 2;
        return true;
    }
    else if (state_ == 2 && monitored_distance > distance_to_door_)
    {
        state_ == 3;
        return true;
    }
    else if (state_ == 3 && monitored_distance > distance_after_passing_)
    {
        state_ == 4;
        return true;
    }
    else      
        return false;
}

int DoorPassing::getState()
{
    return state_;
}

bool DoorPassing::isInitialOrientationCorrect(double monitored_heading)
{
    double diff = fabs(monitored_heading - intial_orientation_);
    if( diff < 0.05)
        return true;
    else
        return false;
}

bool DoorPassing::computeInitialOrientation(int goal, Gateways detected_gateways)
{
    if( goal == 0)
    {
        intial_orientation_ = detected_gateways.left_door.turn_angle - (M_PI/2);
        return true;
    }
    else if (goal == 1)
    {
        intial_orientation_ = detected_gateways.left_door.turn_angle;
        return true;
    }
    else if( goal == 2)
    {
        intial_orientation_ = detected_gateways.right_door.turn_angle + (M_PI/2);
        return true;
    }

    return false;
}

double DoorPassing::getInitialOrientation()
{
    return intial_orientation_;
}

