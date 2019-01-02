// C/C++
#include <algorithm>
#include <vector>
#include <map>
#include <cmath>

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

// Signal handling
#include <signal.h>

// SMCL
#include "smcl/map/semantic_map.h"
#include "smcl/pf/pf.h"
#include "smcl/sensors/odom.h"
#include "smcl/sensors/wall_sides.h"
#include "smcl/sensors/pillars.h"

// roscpp
#include "ros/assert.h"
#include "ros/ros.h"

// Messages that I need
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "std_srvs/Empty.h"
#include "osm_map_msgs/SemanticMap.h"
#include "osm_map_msgs/GetSemanticMap.h"
#include "osm_map_msgs/SetSemanticMap.h"

// For transform support
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/tf.h"

// Pose hypothesis
typedef struct
{
    double weight;                // Total weight (weights sum to 1)
    pf_vector_t pf_pose_mean;     // Mean of pose esimate
    pf_matrix_t pf_pose_cov;      // Covariance of pose estimate

} smcl_hyp_t;

static double normalize(double z)
{
    return atan2(sin(z), cos(z));
}

static double angle_diff(double a, double b)
{
    double d1, d2;
    a = normalize(a);
    b = normalize(b);
    d1 = a - b;
    d2 = 2 * M_PI - fabs(d1);
    if (d1 > 0)
        d2 *= -1.0;
    if (fabs(d1) < fabs(d2))
        return (d1);
    else
        return (d2);
}



class SMCL
{
public:
    SMCL();
    ~SMCL();
    int process();
    void savePoseToServer();

private:
    // Pose-generating function used to uniformly distribute particles over the map
    static pf_vector_t uniformPoseGenerator(void* arg);
    
    // Callbacks
    bool globalLocalizationCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    bool nomotionUpdateCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    bool setMapCallback(osm_map_msgs::SetSemanticMap::Request& req, osm_map_msgs::SetSemanticMap::Response& res);
    void semanticFeaturesReceived(const osm_map_msgs::SemanticMap& semantic_map);
    void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    void handleInitialPoseMessage(const geometry_msgs::PoseWithCovarianceStamped& msg);
    void mapReceived(const osm_map_msgs::SemanticMapConstPtr& msg);
    void odomReceived(const nav_msgs::OdometryConstPtr& odom_msg);

    boost::recursive_mutex configuration_mutex_;

    // other helper functions
    void handleMapMessage(const osm_map_msgs::SemanticMap& msg);
    void freeMapDependentMemory();
    semantic_map_t* convertMap( const osm_map_msgs::SemanticMap& map_msg);
    void updatePoseFromServer();
    void applyInitialPose();

    ros::Timer check_semantic_features_timer_;
    ros::Time last_semantic_features_received_ts_;
    ros::Duration semantic_features_check_interval_;
    void checkSemanticFeaturesReceived(const ros::TimerEvent& event);


    //parameter for what odom to use
    std::string odom_frame_id_;
    //parameter for what base to use
    std::string base_frame_id_;
    //parameter for global frame name (map frame)
    std::string global_frame_id_;

    std::string odom_topic_;
    std::string semantic_features_topic_;
    std::string semantic_map_topic_;


    //paramater to store latest odom pose
    pf_vector_t updated_odom_pose_;
    geometry_msgs::PoseWithCovarianceStamped last_published_pose;

    ros::Duration gui_publish_period;
    ros::Time save_pose_last_time;
    ros::Duration save_pose_period;

    semantic_map_t* semantic_map_;
    bool use_map_topic_;
    bool first_map_only_;
    // TODO @lakshadeep - this requires area ref/id as input
    void requestMap();

    // Particle filter
    pf_t *pf_;
    double pf_err_, pf_z_;
    bool pf_init_;
    pf_vector_t pf_odom_pose_;
    double d_thresh_, a_thresh_;
    int resample_interval_;
    int resample_count_;
    double features_min_range_;
    double features_max_range_;
    int min_particles_, max_particles_;
    double alpha1_, alpha2_, alpha3_, alpha4_, alpha5_;
    double alpha_slow_, alpha_fast_;
    double z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_, chi_outlier_;
    //beam skip related params
    bool do_beamskip_;
    double beam_skip_distance_, beam_skip_threshold_, beam_skip_error_threshold_;
    odom_model_t odom_model_type_;
    double init_pose_[3];
    double init_cov_[3];
    Odom* odom_;

    WallSides* wall_sides_;
    Pillars* pillars_;

    //Nomotion update control
    bool m_force_update;  // used to temporarily let xmcl update samples even when no motion occurs...

    ros::Duration cloud_pub_interval;
    ros::Time last_cloud_pub_time;

    //time for tolerance on the published transform,
    //basically defines how long a map->odom transform is good for
    ros::Duration transform_tolerance_;
    tf::TransformBroadcaster br_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher pose_pub_;
    ros::Publisher particlecloud_pub_;
    ros::ServiceServer global_loc_srv_;
    ros::ServiceServer nomotion_update_srv_; //to let smcl update samples without requiring motion
    ros::ServiceServer set_map_srv_;
    ros::Subscriber initial_pose_sub_old_;
    ros::Subscriber semantic_features_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber initial_pose_sub_;
    ros::Subscriber semantic_map_sub_;

    smcl_hyp_t* initial_pose_hyp_;
    bool first_map_received_;
    bool first_reconfigure_call_;
};