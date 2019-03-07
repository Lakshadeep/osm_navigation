#include <junction_maneuvering/junction_maneuvering.h>
#include <ros/ros.h>

JunctionManeuvering::JunctionManeuvering()
{
    reset();
}

JunctionManeuvering::~JunctionManeuvering()
{
}

bool JunctionManeuvering::setGoal(int goal, double turn_direction, double distance, Gateways detected_gateways)
{
    goal_ = goal;
    distance_ = distance;

    if( goal_ == 0 && turn_direction == 0)
    {
        turn_range_ = detected_gateways.t_junction.left_turn_range * 0.8;
        turn_angle_ = detected_gateways.t_junction.left_turn_angle;
    }
    else if (goal_ == 0 && turn_direction == 2)
    {
        turn_range_ = detected_gateways.t_junction.right_turn_range * 0.6;
        turn_angle_ = detected_gateways.t_junction.right_turn_angle;
    }
    else if (goal_ == 1 && turn_direction == 0)
    {
        turn_range_ = detected_gateways.x_junction.left_turn_range * 0.8;
        turn_angle_ = detected_gateways.x_junction.left_turn_angle * 0.8;
    }
    else if (goal_ == 1 && turn_direction == 2)
    {
        turn_range_ = detected_gateways.x_junction.right_turn_range * 0.6;
        turn_angle_ = detected_gateways.x_junction.right_turn_angle * 0.6;
    }
    else if (goal_ == 1 && turn_direction == 1)
    {
        turn_range_ = detected_gateways.x_junction.front_range;
        turn_angle_ = detected_gateways.x_junction.front_angle;
    }
    else
    { 
        return false;
    }
    return true;
}

double JunctionManeuvering::getTurnAngle()
{
    return turn_angle_;
}
    
double JunctionManeuvering::getPassingOrientation()
{
    return passing_orientation_;
}

bool JunctionManeuvering::isStateChanged(double monitored_distance, double monitored_heading, Gateways detected_gateways)
{
    if (state_ == -1 && monitored_distance > turn_range_)
    {
        state_ = 0;
        return true;
    }
    else if (state_ == 0 && fabs(monitored_heading - turn_angle_) < 0.02)
    {
        state_ = 1;
        return true;
    }
    else if (state_ == 1 && (monitored_distance > distance_))
    {
        state_ = 2;
        return true;
    }
    return false;
}

int JunctionManeuvering::getState()
{
    return state_;
}


bool JunctionManeuvering::computePassingOrientation(Gateways detected_gateways)
{
    if( detected_gateways.hallway.left_range > 0 && detected_gateways.hallway.right_range > 0)
    {
        passing_orientation_ = (detected_gateways.hallway.left_angle + detected_gateways.hallway.right_angle)/2.0;
        return true;
    }
    else if (detected_gateways.hallway.left_range > 0)
    {
        passing_orientation_ = detected_gateways.hallway.left_angle;
        return true;
    }
    else if (detected_gateways.hallway.right_range > 0)
    {
        passing_orientation_ = detected_gateways.hallway.right_angle;
        return true;
    }

    return false;
}

void JunctionManeuvering::reset()
{
    goal_ = -1;
    state_ = -1;

    turn_range_ = 0;
    turn_angle_ = 0;
    
    passing_orientation_ = 0;

    distance_ = 0;

}

