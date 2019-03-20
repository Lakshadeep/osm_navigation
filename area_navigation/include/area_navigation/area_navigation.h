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
    void setRecoveryDirectionThreshold(double recovery_direction_threshold);
    void setCorrectionDirectionThreshold(double correction_direction_threshold);
    void setNominalVelocity(double nominal_velocity);
    bool determineDirection(double &computed_direction, double curr_direction, Gateways gateways, SemanticFeatures semantic_features);
    bool isCorrectDirection(double left_ref_direction, double left_ref_range, double right_ref_direction, double right_ref_range);
    double computeVelocity(double monitored_distance, double monitored_heading);
    void setGoal(int goal, int direction, float distance, std::vector<int> features, std::vector<int> features_directions, 
                 std::vector<float> features_distances);
    void reset();
    double getDesiredDirection();
    double getNominalVelocity();
    double getState();
    bool isStateChanged(double monitored_distance, double monitored_heading, SemanticFeatures semantic_features);
    double getInitialOrientation();

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

    int state_;

    std::vector<int> features_;
    std::vector<int> features_directions_;
    std::vector<float> features_distances_;

    // methods
    WallSide getReferenceWallSide(std::vector<WallSide> wall_sides, double direction, double distance);
    double getWallLength(WallSide ws);
    int angleToDirection(double angle);
    bool computeReferenceDirection(int feature_type, int feature_direction, double feature_distance, 
                                   SemanticFeatures semantic_features, double &computed_direction);
};

#endif
