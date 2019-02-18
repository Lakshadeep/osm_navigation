#ifndef CORRIDOR_NAVIGATION_H
#define CORRIDOR_NAVIGATION_H

#include <vector>
#include <cmath>

class CorridorNavigation
{

public:
    CorridorNavigation();
    ~CorridorNavigation();
private:
    double desired_direction_;
    double recovery_direction_threshold_;
    double correction_direction_threshold_;

    void setRecoveryDirectionThreshold(double recovery_direction_threshold);
    void setCorrectionDirectionThreshold(double correction_direction_threshold);
    bool determineDirection(double &computed_direction, double curr_direction, double left_ref_direction, 
                            double left_ref_range, double right_ref_direction, double right_ref_range);

};

#endif
