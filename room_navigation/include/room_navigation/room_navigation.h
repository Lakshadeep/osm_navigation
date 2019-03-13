#ifndef ROOM_NAVIGATION_H
#define ROOM_NAVIGATION_H

#include <vector>
#include <cmath>
#include <room_navigation/structs.h>

class RoomNavigation
{

public:
    RoomNavigation();
    ~RoomNavigation();
    void setRecoveryDirectionThreshold(double recovery_direction_threshold);
    void setCorrectionDirectionThreshold(double correction_direction_threshold);
    void setNominalVelocity(double nominal_velocity);
    bool determineDirection(double &computed_direction, double curr_direction, Gateways gateways, SemanticFeatures semantic_features);
    bool isCorrectDirection(double left_ref_direction, double left_ref_range, double right_ref_direction, double right_ref_range);
    bool isGoalReached(Gateways detected_gateways, double monitored_distance, double monitored_heading);
    double computeVelocity(double monitored_distance, double monitored_heading);
    void setGoal(int goal, int direction, float distance, std::vector<int> features, std::vector<int> features_directions, 
                 std::vector<float> features_distances);
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
