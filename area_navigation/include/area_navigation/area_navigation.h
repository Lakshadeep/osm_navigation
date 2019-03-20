#ifndef AREA_NAVIGATION_H
#define AREA_NAVIGATION_H

#include <vector>
#include <cmath>
#include <area_navigation/structs.h>

class AreaNavigation
{

public:
    AreaNavigation();
    ~AreaNavigation();

    void setNominalVelocity(double nominal_velocity);
    void setGoal(int goal_type);

    bool determineDirection(double &computed_direction, double curr_direction, std::vector<NavigationSign> navigation_signs);
    double computeVelocity(double monitored_distance, double monitored_heading);
    
    void reset();

    double getDesiredDirection();
    double getNominalVelocity();
    double getState();

    bool isStateChanged(double monitored_distance, double monitored_heading, std::vector<NavigationSign> navigation_signs);

private:
    // params
    double nominal_velocity_;

    // goal 
    double desired_direction_;
    double desired_distance_;
    // 0 : Exit
    int goal_type_; 

    int state_;

    std::vector<NavigationSign> current_navigation_sign_;

    
};

#endif
