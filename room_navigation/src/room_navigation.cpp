#include <room_navigation/room_navigation.h>
#include <ros/ros.h>

RoomNavigation::RoomNavigation():desired_direction_(0), recovery_direction_threshold_(1.0), 
correction_direction_threshold_(0.06), desired_distance_(0), goal_type_(-1), state_(-1)
{
}

RoomNavigation::~RoomNavigation()
{
}

void RoomNavigation::setRecoveryDirectionThreshold(double recovery_direction_threshold)
{
    recovery_direction_threshold_ = recovery_direction_threshold;
}

void RoomNavigation::setCorrectionDirectionThreshold(double correction_direction_threshold)
{
    correction_direction_threshold_ = correction_direction_threshold;
}

void RoomNavigation::setNominalVelocity(double nominal_velocity)
{
    nominal_velocity_ = nominal_velocity;
}

double RoomNavigation::getNominalVelocity()
{
    return nominal_velocity_;
}

double RoomNavigation::getDesiredDirection()
{
    return desired_direction_; 
}

double RoomNavigation::getInitialOrientation()
{
    if (desired_direction_ == 0)
        return (M_PI/2.0);
    else if (desired_direction_ == 2)
        return (-M_PI/2.0);
    return 0;
}

double RoomNavigation::getState()
{
    return state_;
}

bool RoomNavigation::determineDirection(double &computed_direction, double curr_direction, Gateways gateways,
                                        SemanticFeatures semantic_features)
{
    bool status = false;
    for (int i = 0; i < features_.size(); i++)
    {
        status = status || computeReferenceDirection(features_[i], features_directions_[i], features_distances_[i], 
                                           semantic_features, computed_direction);
    }
    computed_direction = computed_direction + curr_direction;
    return status;
}

// NOTE: this function assumes that goal direction is in the front
bool RoomNavigation::computeReferenceDirection(int feature_type, int feature_direction, double feature_distance, 
                                               SemanticFeatures semantic_features, double &computed_direction)
{
    if (feature_type == 1)
    {
        WallSide ws = getReferenceWallSide(semantic_features.wall_sides, feature_direction, feature_distance);
        if (ws.radius > 0)
        {
            if (feature_direction == 0)
                computed_direction = ws.angle - (M_PI/2.0);
            else if (feature_direction == 2)
                computed_direction == ws.angle + (M_PI/2.0);
            return true;
        }
        else
        {
            ROS_ERROR("No reference wall found");
            return false;
        }
    }
    else
    {
        ROS_ERROR("Only walls can be used for computing reference orientation");
        return false;
    }
}

WallSide RoomNavigation::getReferenceWallSide(std::vector<WallSide> wall_sides, double direction, double distance)
{
    std::vector<WallSide> possible_wall_sides;
    for (int i = 0; i < wall_sides.size(); i++)
    {
        if (getWallLength(wall_sides[i]) > 1.0 && wall_sides[i].radius > (0.8*distance) && wall_sides[i].radius < (1.2*distance))
        {
            possible_wall_sides.push_back(wall_sides[i]);
        }
    }

    WallSide ws;

    for (int i = 0; i < possible_wall_sides.size(); i++)
    {
        if (i == 0)
        {
            ws.corner1 = possible_wall_sides[i].corner1;
            ws.corner2 = possible_wall_sides[i].corner2;
        }
        else
            ws.corner2 = possible_wall_sides[i].corner2;

        ws.radius = ws.radius + possible_wall_sides[i].radius;
        ws.angle = ws.angle + possible_wall_sides[i].angle;
    }

    if (possible_wall_sides.size() > 0)
    {
        ws.radius = ws.radius/possible_wall_sides.size();
        ws.angle = ws.angle/possible_wall_sides.size();
    }
    else
    {
        ws.radius = 0;
        ws.angle = 0;
    }
    return ws;
}

double RoomNavigation::getWallLength(WallSide ws)
{
    return pow(pow(ws.corner1.x - ws.corner2.x, 2) + pow(ws.corner1.y - ws.corner2.y, 2) ,0.5);
}

int RoomNavigation::angleToDirection(double angle)
{
    if (angle < (3*M_PI/4) && angle > (M_PI/4)) 
        return 0;
    else if (angle < (M_PI/4) && angle > (-M_PI/4))
        return 1;
    else if (angle < (-M_PI/4) && angle > (-3*M_PI/4))
        return 2;
}

bool RoomNavigation::isCorrectDirection(double left_ref_direction, double left_ref_range, 
                                            double right_ref_direction, double right_ref_range)
{
    if(left_ref_range > 0  && right_ref_range > 0)
    {
        if (fabs(left_ref_direction - right_ref_direction) < correction_direction_threshold_)
            return true;
    }
    return false;
}

double RoomNavigation::computeVelocity(double monitored_distance, double monitored_heading)
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

void RoomNavigation::setGoal(int goal, int direction, float distance, std::vector<int> features, std::vector<int> features_directions, 
                            std::vector<float> features_distances)
{
    goal_type_ = goal;
    desired_direction_ = direction;
    desired_distance_ = distance;
    features_ = features;
    features_directions_ = features_directions;
    features_distances_ = features_distances;
}

void RoomNavigation::reset()
{
    desired_direction_ = 0;
    desired_distance_ = 0;
    goal_type_ = -1;
    state_ = -1;
    features_.clear();
    features_directions_.clear();
    features_distances_.clear();
}

bool RoomNavigation::isStateChanged(double monitored_distance, double monitored_heading, SemanticFeatures semantic_features)
{
    if (state_ == -1 && desired_direction_ == 1)
    {
        state_ = 0;
        return true;
    }
    else if(state_ == -1 && desired_direction_ == 0 && monitored_heading > (M_PI/2.0)) 
    {
        state_ = 0;
        return true;
    }
    else if(state_ == -1 && desired_direction_ == 2 && monitored_heading < (-M_PI/2.0)) 
    {
        state_ = 0;
        return true;
    }
    else if(state_ == 0 && monitored_distance > desired_distance_)
    {
        state_ = 1;
        return true;
    }
    return false;
}
