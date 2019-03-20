#include <area_navigation/area_navigation.h>
#include <ros/ros.h>

AreaNavigation::AreaNavigation():desired_direction_(0), desired_distance_(0), goal_type_(-1), state_(-1)
{
}

AreaNavigation::~AreaNavigation()
{
}

void AreaNavigation::setNominalVelocity(double nominal_velocity)
{
    nominal_velocity_ = nominal_velocity;
}

double AreaNavigation::getNominalVelocity()
{
    return nominal_velocity_;
}

double AreaNavigation::getDesiredDirection()
{
    return desired_direction_; 
}

double AreaNavigation::getState()
{
    return state_;
}

bool AreaNavigation::determineDirection(double &computed_direction, double curr_direction, std::vector<NavigationSign> navigation_signs)
{
    bool status = false;
    
    return status;
}

double AreaNavigation::computeVelocity(double monitored_distance, double monitored_heading)
{
    double velocity = nominal_velocity_;

    if( monitored_distance > (0.8 * desired_distance_) && monitored_distance < (1.2 * desired_distance_))
    {
        velocity = 0.7 * nominal_velocity_;
    }  

    if (monitored_heading < (desired_direction_ - 0.4) || monitored_heading > (desired_direction_ + 0.4))
    {
        velocity = 0.5 * nominal_velocity_;
    }
    
    return velocity;
}

void AreaNavigation::setGoal(int goal_type)
{
    goal_type_ = goal_type;
}

void AreaNavigation::reset()
{
    desired_direction_ = 1;
    desired_distance_ = 0;
    goal_type_ = -1;
    state_ = -1;
}

bool AreaNavigation::isStateChanged(double monitored_distance, double monitored_heading, std::vector<NavigationSign> navigation_signs)
{
    if (state_ == -1)
    {
        state_ = 0;
        return true;
    }
    else if(state_ == -1) 
    {
        state_ = 0;
        return true;
    }
    else if(state_ == -1) 
    {
        state_ = 0;
        return true;
    }
    else if(state_ == 0)
    {
        state_ = 1;
        return true;
    }
    return false;
}
