#include <corridor_navigation/corridor_navigation.h>

CorridorNavigation::CorridorNavigation():desired_direction_(0), recovery_direction_threshold_(0.5), 
correction_direction_threshold_(0.06)
{
}

CorridorNavigation::~CorridorNavigation()
{
}

void CorridorNavigation::setRecoveryDirectionThreshold(double recovery_direction_threshold)
{
    recovery_direction_threshold_ = recovery_direction_threshold;
}

void CorridorNavigation::setCorrectionDirectionThreshold(double correction_direction_threshold)
{
    correction_direction_threshold_ = correction_direction_threshold;
}

bool CorridorNavigation::determineDirection(double &computed_direction, double curr_direction, double left_ref_direction, 
                                            double left_ref_range, double right_ref_direction, double right_ref_range)
{
       
    if (fabs(curr_direction) < recovery_direction_threshold_)
    { 
        // NOTE: both the ranges greater then 0 indicates both are valid ref angles 
        if(left_ref_range > 0  && right_ref_range > 0)
        {
            computed_direction = desired_direction_ + (left_ref_direction + right_ref_direction)/2.0;

            if (fabs(left_ref_direction - right_ref_direction) < correction_direction_threshold_)
                desired_direction_ = (left_ref_direction + right_ref_direction)/2.0;
        }
        else if (left_ref_range > 0)
            computed_direction = left_ref_direction;
        else if (right_ref_range > 0)
            computed_direction = right_ref_direction;
        return true;
    }
    else
        return false;
      // enable recovery behaviour i.e stop the monitors until robot comes back in valid direction range
}