#ifndef CORRIDOR_NAVIGATION_H
#define CORRIDOR_NAVIGATION_H

#include <vector>
#include <cmath>
#include <corridor_navigation/structs.h>

class CorridorNavigation
{

public:
    CorridorNavigation();
    ~CorridorNavigation();
    void setRecoveryDirectionThreshold(double recovery_direction_threshold);
    void setCorrectionDirectionThreshold(double correction_direction_threshold);
    void setNominalVelocity(double nominal_velocity);
    bool determineDirection(double &computed_direction, double curr_direction, double left_ref_direction, 
                            double left_ref_range, double right_ref_direction, double right_ref_range);
    bool isCorrectDirection(double left_ref_direction, double left_ref_range, double right_ref_direction, double right_ref_range);
    bool isGoalReached(Gateways detected_gateways, double monitored_distance, double monitored_heading);
    double computeVelocity(double monitored_distance, double monitored_heading);
    void setGoal(int goal, double direction, double distance);
    void reset();
    double getDesiredDirection();

private:
    // params
    double recovery_direction_threshold_;
    double correction_direction_threshold_;
    double nominal_velocity_;

    // goal 
    double desired_direction_;
    double desired_distance_;
    // 0 : T-junction | 1 : X-junction | 2 : left door | 3 : right door
    int goal_type_;



};

#endif
