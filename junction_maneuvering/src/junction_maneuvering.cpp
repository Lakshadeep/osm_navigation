#include <junction_maneuvering/junction_maneuvering.h>
#include <ros/ros.h>

JunctionManeuvering::JunctionManeuvering()
{
    reset();
}

JunctionManeuvering::~JunctionManeuvering()
{
}

bool JunctionManeuvering::setGoal(int goal, int turn_direction, double distance, Gateways detected_gateways)
{
    goal_ = goal;
    distance_ = distance;
    turn_direction_ = turn_direction;

    if( goal_ == 0 && turn_direction == 0)
    {
        turn_range_ = (0.8 * detected_gateways.t_junction.left_turn_range) - 0.75;
        turn_angle_ = detected_gateways.t_junction.left_turn_angle;
    }
    else if (goal_ == 0 && turn_direction == 2)
    {
        turn_range_ = (0.8 * detected_gateways.t_junction.right_turn_range) - 0.5;
        turn_angle_ = detected_gateways.t_junction.right_turn_angle;
    }
    else if (goal_ == 1 && turn_direction == 0)
    {
        turn_range_ = (0.8 * detected_gateways.x_junction.left_turn_range)  - 0.75;
        turn_angle_ = detected_gateways.x_junction.left_turn_angle;
    }
    else if (goal_ == 1 && turn_direction == 2)
    {
        turn_range_ = (0.8 * detected_gateways.x_junction.right_turn_range) - 0.5;
        turn_angle_ = detected_gateways.x_junction.right_turn_angle;
    }
    else if (goal_ == 1 && turn_direction == 1)
    {
        turn_range_x_ = detected_gateways.x_junction.front_range_x;
        turn_range_y_ = detected_gateways.x_junction.front_range_y;
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

double JunctionManeuvering::getInitialOrientation()
{
    return initial_orientation_;
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

bool JunctionManeuvering::computeInitialOrientation(Gateways detected_gateways)
{
    if (goal_ == 1 && turn_direction_ == 1)
    {
        initial_orientation_ = atan2(detected_gateways.x_junction.front_range_y, detected_gateways.x_junction.front_range_x);
        return true;
    }

    return false;
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
    turn_range_x_ = 0;
    turn_range_y_ = 0;
    turn_angle_ = 0;
    
    initial_orientation_ = 0;
    passing_orientation_ = 0;

    distance_ = 0;

}

