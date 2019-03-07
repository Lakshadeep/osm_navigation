#include <junction_maneuvering/junction_maneuvering.h>
#include <ros/ros.h>

JunctionManeuvering::JunctionManeuvering():laser_robot_center_offset_x_(0.3)
{
    reset();
}

JunctionManeuvering::~JunctionManeuvering()
{
}

bool JunctionManeuvering::setGoal(int goal, double distance,  Gateways detected_gateways)
{
    goal_ = goal;
    distance_inside_ = distance;
    if( goal_ == 0)
    {
        turn_range_ = detected_gateways.left_door.range_x;
        turn_angle_ = detected_gateways.left_door.angle;
    }
    else if (goal_ == 1)
    {
        turn_range_ = detected_gateways.front_door.range_x;
        turn_angle_ = detected_gateways.front_door.angle;
    }
    else if (goal_ == 2)
    {
        turn_range_ = detected_gateways.right_door.range_x;
        turn_angle_ = detected_gateways.right_door.angle;
    }
    else
    { 
        return false;
    }
    return true;
}

void JunctionManeuvering::setParams(double laser_robot_center_offset_x)
{
    laser_robot_center_offset_x_ = laser_robot_center_offset_x;
}
    
double JunctionManeuvering::getInitialOrientation()
{
    return intial_orientation_;
}

double JunctionManeuvering::getTurnAngle()
{
    return turn_angle_;
}
    
double JunctionManeuvering::getPassingOrientation()
{
    return passing_orientation_;
}

double JunctionManeuvering::getInsideOrientation()
{
    return orientation_inside_;
}

bool JunctionManeuvering::isStateChanged(double monitored_distance, double monitored_heading, Gateways detected_gateways)
{
    if (state_ == -1 && fabs(monitored_heading - intial_orientation_) < 0.02)
    {
        if (goal_ == 1)
        {
            state_ = 2;
            computePassingOrientation(detected_gateways);
            computeDistanceToDoor(detected_gateways);
        }
        else
            state_ = 0;
        return true;
    }
    else if (state_ == 0 && (monitored_distance) > turn_range_)
    {
        state_ = 1;
        return true;
    }
    else if (state_ == 1 && fabs(monitored_heading - turn_angle_) < 0.02)
    {
        state_ = 2;
        computePassingOrientation(detected_gateways);
        computeDistanceToDoor(detected_gateways);
        return true;
    }
    else if (state_ == 2 && (monitored_distance > distance_to_door_))
    {
        state_ = 3;
        return true;
    }
    else if (state_ == 3 && (monitored_distance > distance_inside_))
    {
        state_ = 4;
        return true;
    }
    return false;
}

int JunctionManeuvering::getState()
{
    return state_;
}

bool JunctionManeuvering::computeInitialOrientation(int goal, Gateways detected_gateways)
{
    if( goal == 0 && detected_gateways.left_door.range_x > 0)
    {
        intial_orientation_ = detected_gateways.left_door.angle - (M_PI/2);
        return true;
    }
    else if (goal == 1 && detected_gateways.front_door.range_x > 0)
    {
        intial_orientation_ = detected_gateways.front_door.angle;
        return true;
    }
    else if( goal == 2 && detected_gateways.right_door.range_x > 0)
    {
        intial_orientation_ = detected_gateways.right_door.angle + (M_PI/2);
        return true;
    }

    return false;
}

bool JunctionManeuvering::computePassingOrientation(Gateways detected_gateways)
{
    if( detected_gateways.front_door.range_x > 0)
    {
        passing_orientation_ = atan2(detected_gateways.front_door.range_y, detected_gateways.front_door.range_x);
        return true;
    }

    return false;
}

bool JunctionManeuvering::computeDistanceToDoor(Gateways detected_gateways)
{
    if( detected_gateways.front_door.range_x > 0)
    {
        distance_to_door_ = pow(pow(detected_gateways.front_door.range_x, 2) + pow(detected_gateways.front_door.range_y, 2), 0.5);
        return true;
    }

    return false;
}

bool JunctionManeuvering::computeInsideOrientation(Gateways detected_gateways)
{
    if( detected_gateways.hallway.left_range > 0 && detected_gateways.hallway.right_range > 0)
    {
        orientation_inside_ = (detected_gateways.hallway.left_angle + detected_gateways.hallway.right_angle)/2.0;
        return true;
    }
    else if (detected_gateways.hallway.left_range > 0)
    {
        orientation_inside_ = detected_gateways.hallway.left_angle;
        return true;
    }
    else if (detected_gateways.hallway.right_range > 0)
    {
        orientation_inside_ = detected_gateways.hallway.right_angle;
        return true;
    }
    return false;
}

void JunctionManeuvering::reset()
{
    goal_ = -1;
    state_ = -1;

    intial_orientation_ = 0;

    turn_range_ = 0;
    turn_angle_ = 0;
    
    distance_to_door_ = 0;
    passing_orientation_ = 0;

    distance_inside_ = 0;
    orientation_inside_ = 0;
}

