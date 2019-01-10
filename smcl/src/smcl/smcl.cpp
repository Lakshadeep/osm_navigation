#include "smcl/smcl.h"

SMCL::SMCL() : semantic_map_(NULL), pf_(NULL), resample_count_(0), odom_(NULL), private_nh_("~"), initial_pose_hyp_(NULL)
{
    boost::recursive_mutex::scoped_lock l(configuration_mutex_);

    double tmp;
    private_nh_.param("gui_publish_rate", tmp, -1.0);
    gui_publish_period = ros::Duration(1.0 / tmp);
    private_nh_.param("save_pose_rate", tmp, 0.5);
    save_pose_period = ros::Duration(1.0 / tmp);

    private_nh_.param("features_min_range", features_min_range_, -1.0);
    private_nh_.param("features_max_range", features_max_range_, -1.0);
    private_nh_.param("min_particles", min_particles_, 100);
    private_nh_.param("max_particles", max_particles_, 5000);
    private_nh_.param("kld_err", pf_err_, 0.01);
    private_nh_.param("kld_z", pf_z_, 0.99);
    private_nh_.param("odom_alpha1", alpha1_, 0.2);
    private_nh_.param("odom_alpha2", alpha2_, 0.2);
    private_nh_.param("odom_alpha3", alpha3_, 0.2);
    private_nh_.param("odom_alpha4", alpha4_, 0.2);
    private_nh_.param("odom_alpha5", alpha5_, 0.2);

    private_nh_.param("do_beamskip", do_beamskip_, false);
    private_nh_.param("beam_skip_distance", beam_skip_distance_, 0.5);
    private_nh_.param("beam_skip_threshold", beam_skip_threshold_, 0.3);
    if (private_nh_.hasParam("beam_skip_error_threshold_"))
    {
        private_nh_.param("beam_skip_error_threshold_", beam_skip_error_threshold_);
    }
    else
    {
        private_nh_.param("beam_skip_error_threshold", beam_skip_error_threshold_, 0.9);
    }

    private_nh_.param("features_z_hit", z_hit_, 0.95);
    private_nh_.param("features_z_short", z_short_, 0.1);
    private_nh_.param("features_z_max", z_max_, 0.05);
    private_nh_.param("features_z_rand", z_rand_, 0.05);
    private_nh_.param("features_sigma_hit", sigma_hit_, 0.2);
    private_nh_.param("features_lambda_short", lambda_short_, 0.1);

    std::string tmp_model_type;
    private_nh_.param("odom_model_type", tmp_model_type, std::string("diff"));
    if (tmp_model_type == "diff")
        odom_model_type_ = ODOM_MODEL_DIFF;
    else if (tmp_model_type == "omni")
        odom_model_type_ = ODOM_MODEL_OMNI;
    else if (tmp_model_type == "diff-corrected")
        odom_model_type_ = ODOM_MODEL_DIFF_CORRECTED;
    else if (tmp_model_type == "omni-corrected")
        odom_model_type_ = ODOM_MODEL_OMNI_CORRECTED;
    else
    {
        ROS_WARN("Unknown odom model type \"%s\"; defaulting to diff model",
                 tmp_model_type.c_str());
        odom_model_type_ = ODOM_MODEL_DIFF;
    }

    private_nh_.param("update_min_d", d_thresh_, 0.1);
    private_nh_.param("update_min_a", a_thresh_, M_PI / 6.0);
    private_nh_.param("odom_frame_id", odom_frame_id_, std::string("odom"));
    private_nh_.param("base_frame_id", base_frame_id_, std::string("base_link"));
    private_nh_.param("global_frame_id", global_frame_id_, std::string("map"));
    private_nh_.param("resample_interval", resample_interval_, 2);

    private_nh_.param("odom_topic", odom_topic_, std::string("odom"));
    private_nh_.param("semantic_features_topic", semantic_features_topic_, std::string("semantic_features"));
    private_nh_.param("semantic_map_topic", semantic_map_topic_, std::string("semantic_map"));

    double tmp_tol;
    private_nh_.param("transform_tolerance", tmp_tol, 0.1);
    private_nh_.param("recovery_alpha_slow", alpha_slow_, 0.001);
    private_nh_.param("recovery_alpha_fast", alpha_fast_, 0.1);

    transform_tolerance_.fromSec(tmp_tol);

    updatePoseFromServer();

    cloud_pub_interval.fromSec(1.0);

    pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("smcl_pose", 2, true);
    particlecloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particlecloud", 2, true);
    global_loc_srv_ = nh_.advertiseService("global_localization", &SMCL::globalLocalizationCallback, this);
    nomotion_update_srv_ = nh_.advertiseService("request_nomotion_update", &SMCL::nomotionUpdateCallback, this);
    semantic_features_sub_ = nh_.subscribe(semantic_features_topic_, 10, &SMCL::semanticFeaturesReceived, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 10, &SMCL::odomReceived, this);
    initial_pose_sub_ = nh_.subscribe("initialpose", 2, &SMCL::initialPoseReceived, this);
    semantic_map_sub_ = nh_.subscribe(semantic_map_topic_, 1, &SMCL::mapReceived, this);
    ROS_INFO("Subscribed to map topic.");

    m_force_update = false;

    semantic_features_check_interval_ = ros::Duration(15.0);
    check_semantic_features_timer_ = nh_.createTimer(semantic_features_check_interval_,
                                     boost::bind(&SMCL::checkSemanticFeaturesReceived, this, _1));
}

void SMCL::updatePoseFromServer()
{
    init_pose_[0] = 0.0;
    init_pose_[1] = 0.0;
    init_pose_[2] = 0.0;
    init_cov_[0] = 0.5 * 0.5;
    init_cov_[1] = 0.5 * 0.5;
    init_cov_[2] = (M_PI / 12.0) * (M_PI / 12.0);
    // Check for NAN on input from param server, #5239
    double tmp_pos;
    private_nh_.param("initial_pose_x", tmp_pos, init_pose_[0]);
    if (!std::isnan(tmp_pos))
        init_pose_[0] = tmp_pos;
    else
        ROS_WARN("ignoring NAN in initial pose X position");
    private_nh_.param("initial_pose_y", tmp_pos, init_pose_[1]);
    if (!std::isnan(tmp_pos))
        init_pose_[1] = tmp_pos;
    else
        ROS_WARN("ignoring NAN in initial pose Y position");
    private_nh_.param("initial_pose_a", tmp_pos, init_pose_[2]);
    if (!std::isnan(tmp_pos))
        init_pose_[2] = tmp_pos;
    else
        ROS_WARN("ignoring NAN in initial pose Yaw");
    private_nh_.param("initial_cov_xx", tmp_pos, init_cov_[0]);
    if (!std::isnan(tmp_pos))
        init_cov_[0] = tmp_pos;
    else
        ROS_WARN("ignoring NAN in initial covariance XX");
    private_nh_.param("initial_cov_yy", tmp_pos, init_cov_[1]);
    if (!std::isnan(tmp_pos))
        init_cov_[1] = tmp_pos;
    else
        ROS_WARN("ignoring NAN in initial covariance YY");
    private_nh_.param("initial_cov_aa", tmp_pos, init_cov_[2]);
    if (!std::isnan(tmp_pos))
        init_cov_[2] = tmp_pos;
    else
        ROS_WARN("ignoring NAN in initial covariance AA");
}

void SMCL::savePoseToServer()
{
    double roll, pitch, yaw;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(last_published_pose.pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    private_nh_.setParam("initial_pose_x", last_published_pose.pose.pose.position.x);
    private_nh_.setParam("initial_pose_y", last_published_pose.pose.pose.position.y);
    private_nh_.setParam("initial_pose_a", yaw);
    private_nh_.setParam("initial_cov_xx", last_published_pose.pose.covariance[6 * 0 + 0]);
    private_nh_.setParam("initial_cov_yy", last_published_pose.pose.covariance[6 * 1 + 1]);
    private_nh_.setParam("initial_cov_aa", last_published_pose.pose.covariance[6 * 5 + 5]);
}

void SMCL::checkSemanticFeaturesReceived(const ros::TimerEvent& event)
{
    ros::Duration d = ros::Time::now() - last_semantic_features_received_ts_;
    if (d > semantic_features_check_interval_)
    {
        ROS_WARN("No semantic features received (and thus no pose updates have been published) for %f seconds.  Verify that data is being published on the %s topic.", d.toSec(), ros::names::resolve(semantic_features_topic_).c_str());
    }
}

void SMCL::mapReceived(const osm_map_msgs::SemanticMapConstPtr& msg)
{
    handleMapMessage( *msg );
}

void SMCL::handleMapMessage(const osm_map_msgs::SemanticMap& msg)
{
    boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);

    ROS_INFO("Received semantic map");

    freeMapDependentMemory();

    semantic_map_ = convertMap(msg);

    ROS_INFO("Semantic map successfully converted");
    // Create the particle filter
    pf_ = pf_alloc(min_particles_, max_particles_, alpha_slow_, alpha_fast_, (pf_init_model_fn_t)SMCL::uniformPoseGenerator, (void *)semantic_map_);
    pf_->pop_err = pf_err_;
    pf_->pop_z = pf_z_;

    ROS_INFO("Particle filter intiialized");

    pf_init_model(pf_, (pf_init_model_fn_t)SMCL::uniformPoseGenerator, (void *)semantic_map_);
    pf_init_ = false;

    pf_sample_set_t* set = pf_->sets;
    geometry_msgs::PoseArray cloud_msg;
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = global_frame_id_;
    cloud_msg.poses.resize(set->sample_count);
    for (int i = 0; i < set->sample_count; i++)
    {
        tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(set->samples[i].pose.v[2]), tf::Vector3(set->samples[i].pose.v[0], set->samples[i].pose.v[1], 0)), cloud_msg.poses[i]);
    }
    particlecloud_pub_.publish(cloud_msg);

    // Instantiate the sensor objects
    // Odometry
    delete odom_;
    odom_ = new Odom();
    ROS_ASSERT(odom_);
    odom_->SetModel( odom_model_type_, alpha1_, alpha2_, alpha3_, alpha4_, alpha5_ );

    delete wall_sides_;
    wall_sides_ = new WallSides();
    ROS_ASSERT(wall_sides_);
    wall_sides_->setSensorParams(0.1, 5.0, -M_PI/2.0, -M_PI/2.0);
    wall_sides_->setModelParams(z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_, chi_outlier_);
    wall_sides_->updateMap(semantic_map_);

    delete pillars_;
    pillars_ = new Pillars();
    ROS_ASSERT(pillars_);
    pillars_->setSensorParams(0.1, 5.0, -M_PI/2.0, -M_PI/2.0);
    pillars_->setModelParams(z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_, chi_outlier_);
    pillars_->updateMap(semantic_map_);

    applyInitialPose();
}

void SMCL::freeMapDependentMemory()
{
    if ( semantic_map_ != NULL ) {
        semantic_map_free( semantic_map_ );
        semantic_map_ = NULL;
    }
    if ( pf_ != NULL ) {
        pf_free( pf_ );
        pf_ = NULL;
    }
    delete odom_;
    odom_ = NULL;
}

/**
 * Convert an Semantic map message into the internal
 * representation.  This allocates a semantic_map_t and returns it.
 */
semantic_map_t* SMCL::convertMap( const osm_map_msgs::SemanticMap& semantic_map_msg)
{
    semantic_map_t* map = semantic_map_alloc();
    ROS_ASSERT(map);
    for (auto wall_side_it = semantic_map_msg.wall_sides.begin(); wall_side_it != semantic_map_msg.wall_sides.end(); wall_side_it++)
    {
        for (auto corner_it = wall_side_it->corners.begin(); corner_it != wall_side_it->corners.end(); corner_it++)
        {
            map->min_x = map->min_x == 0 ? corner_it->x : std::min(map->min_x, (float)corner_it->x);
            map->min_y = map->min_y == 0 ? corner_it->y : std::min(map->min_y, (float)corner_it->y);
            map->max_x = map->max_x == 0 ? corner_it->x : std::max(map->max_x, (float)corner_it->x);
            map->max_y = map->max_y == 0 ? corner_it->y : std::max(map->max_y, (float)corner_it->y);
        }
    }

    map->no_of_wall_sides = semantic_map_msg.wall_sides.size();
    map->no_of_door_sides = semantic_map_msg.door_sides.size();
    map->no_of_pillars = semantic_map_msg.pillars.size();
    map->no_of_features = semantic_map_msg.features.size();

    ROS_INFO("No of wall sides in a map: %d", map->no_of_wall_sides);
    ROS_INFO("No of door sides in a map: %d", map->no_of_door_sides);
    ROS_INFO("No of pillars in a map: %d", map->no_of_pillars);
    ROS_INFO("No of other features in a map: %d", map->no_of_features);


    map->wall_sides = (wall_side_t*)malloc(map->no_of_wall_sides * sizeof(wall_side_t));
    ROS_ASSERT(map->wall_sides);
    map->door_sides = (door_side_t*)malloc(map->no_of_door_sides * sizeof(door_side_t));
    ROS_ASSERT(map->door_sides);
    map->pillars = (pillar_t*)malloc(map->no_of_pillars * sizeof(pillar_t));
    ROS_ASSERT(map->pillars);
    map->features = (feature_t*)malloc(map->no_of_features * sizeof(feature_t));
    ROS_ASSERT(map->features);


    for (int i = 0; i < semantic_map_msg.wall_sides.size(); i++)
    {
        point_t pt;
        pt.x = semantic_map_msg.wall_sides[i].corners[0].x;
        pt.y = semantic_map_msg.wall_sides[i].corners[0].y;
        map->wall_sides[i].corner1 = pt;

        pt.x = semantic_map_msg.wall_sides[i].corners[1].x;
        pt.y = semantic_map_msg.wall_sides[i].corners[1].y;
        map->wall_sides[i].corner2 = pt;
    }

    for (int i = 0; i < semantic_map_msg.door_sides.size(); i++)
    {
        point_t pt;
        pt.x = semantic_map_msg.door_sides[i].corners[0].x;
        pt.y = semantic_map_msg.door_sides[i].corners[0].y;
        map->door_sides[i].corner1 = pt;

        pt.x = semantic_map_msg.door_sides[i].corners[1].x;
        pt.y = semantic_map_msg.door_sides[i].corners[1].y;
        map->door_sides[i].corner2 = pt;
    }

    for (int i = 0; i < semantic_map_msg.features.size(); i++)
    {
        point_t pt;
        pt.x = semantic_map_msg.features[i].position.x;
        pt.y = semantic_map_msg.features[i].position.y;
        map->features[i].position = pt;
    }

    for (int i = 0; i < semantic_map_msg.pillars.size(); i++)
    {
        map->pillars[i].corners = (point_t*) NULL;
        map->pillars[i].no_of_corners = semantic_map_msg.pillars[i].shape.points.size();
        map->pillars[i].corners = (point_t*)malloc(map->pillars[i].no_of_corners * sizeof(point_t));
        ROS_ASSERT(map->pillars[i].corners);

        for (int j = 0; j < map->pillars[i].no_of_corners; j++)
        {
            point_t pt;
            pt.x = semantic_map_msg.pillars[i].shape.points[j].x;
            pt.y = semantic_map_msg.pillars[i].shape.points[j].y;
            map->pillars[i].corners[j] = pt;
        }
        map->pillars[i].point.x = semantic_map_msg.pillars[i].point.x; 
        map->pillars[i].point.y = semantic_map_msg.pillars[i].point.y; 
    }
    return map;
}


SMCL::~SMCL()
{
    freeMapDependentMemory();
    // TODO: delete everything allocated in constructor
}

pf_vector_t SMCL::uniformPoseGenerator(void* arg)
{
    semantic_map_t* map = (semantic_map_t*)arg;
    pf_vector_t p;

    p.v[0] = map->min_x + drand48() * (map->max_x - map->min_x);
    p.v[1] = map->min_y + drand48() * (map->max_y - map->min_y);
    p.v[2] = drand48() * 2 * M_PI - M_PI;

    // TODO: check if sample is valid
    return p;
}


void SMCL::odomReceived(const nav_msgs::OdometryConstPtr& odom_msg)
{
    updated_odom_pose_.v[0] = odom_msg->pose.pose.position.x;
    updated_odom_pose_.v[1] = odom_msg->pose.pose.position.y;

    double roll, pitch, yaw;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(odom_msg->pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    updated_odom_pose_.v[2] = yaw;
}

void SMCL::semanticFeaturesReceived(const osm_map_msgs::SemanticMap& semantic_map)
{
    last_semantic_features_received_ts_ = ros::Time::now();
    if ( semantic_map_ == NULL ) 
    {
        return;
    }

    // we take latest odom pose and semantic features
    // better way will be to get odom using time stamp in semantic feaztures msg
    pf_vector_t pose = updated_odom_pose_;

    pf_vector_t delta = pf_vector_zero();

    if (pf_init_)
    {
        // Compute change in pose
        delta.v[0] = pose.v[0] - pf_odom_pose_.v[0];
        delta.v[1] = pose.v[1] - pf_odom_pose_.v[1];
        delta.v[2] = angle_diff(pose.v[2], pf_odom_pose_.v[2]);

        // we update filter only when robot moves
        //bool update = fabs(delta.v[0]) > d_thresh_ || fabs(delta.v[1]) > d_thresh_ || fabs(delta.v[2]) > a_thresh_;
        bool update = true;

        PillarsData pdata;
        pdata.pillars_count = semantic_map.pillars.size();
        pdata.detected_pillars = (pillar_sensor_t*)malloc(pdata.pillars_count * sizeof(pillar_sensor_t));

        WallSidesData wsdata;
        wsdata.wall_sides_count = semantic_map.wall_sides.size();
        wsdata.detected_wall_sides = (wall_side_sensor_t*)malloc(wsdata.wall_sides_count * sizeof(wall_side_sensor_t));

        OdomData odata;
        odata.pose = pose;
        odata.delta = delta;

        for (int i = 0; i < semantic_map.wall_sides.size(); i++)
        {
            point_t pt;
            pt.x = semantic_map.wall_sides[i].corners[0].x;
            pt.y = semantic_map.wall_sides[i].corners[0].y;
            wsdata.detected_wall_sides[i].corner1 = pt;

            pt.x = semantic_map.wall_sides[i].corners[1].x;
            pt.y = semantic_map.wall_sides[i].corners[1].y;
            wsdata.detected_wall_sides[i].corner2 = pt;

            wsdata.detected_wall_sides[i].radius = semantic_map.wall_sides[i].radius;
            wsdata.detected_wall_sides[i].angle = semantic_map.wall_sides[i].angle;
        }
        wsdata.sensor = wall_sides_;

        for (int i = 0; i < semantic_map.pillars.size(); i++)
        {
            point_t pt;
            pt.x = semantic_map.pillars[i].point.x;
            pt.y = semantic_map.pillars[i].point.y;
            pdata.detected_pillars[i].point = pt;

            pdata.detected_pillars[i].radius = semantic_map.pillars[i].radius;
            pdata.detected_pillars[i].angle = semantic_map.pillars[i].angle;
        }
        pdata.sensor = pillars_;

        if (update)
        {
            odom_->UpdateAction(pf_, (SensorData*)&odata);
            pf_free_samples_features(pf_->sets + pf_->current_set);  // deletes samples observed previously
            wall_sides_->UpdateSensor(pf_, (SensorData*)&wsdata);
            pillars_->UpdateSensor(pf_, (SensorData*)&pdata);

            if ((resample_count_ % 50) == 0)
            { 
                pf_re_orient_samples(pf_);
            }
            pf_update_sensor_weights_and_params(pf_);
            // Resample the particles
            if ((resample_count_ % 10) == 0)
            { 
                pf_update_resample_semantic(pf_);
            }

            pf_sample_set_t* set = pf_->sets;
            geometry_msgs::PoseArray cloud_msg;
            cloud_msg.header.stamp = ros::Time::now();
            cloud_msg.header.frame_id = global_frame_id_;
            cloud_msg.poses.resize(set->sample_count);
            for (int i = 0; i < set->sample_count; i++)
            {
                tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(set->samples[i].pose.v[2]),
                                         tf::Vector3(set->samples[i].pose.v[0],
                                                     set->samples[i].pose.v[1], 0)),
                                cloud_msg.poses[i]);
            }
            particlecloud_pub_.publish(cloud_msg);
            pf_odom_pose_ = pose;

            /////////////////////////////////////////////////////////////////////

            double max_weight = 0.0;
            int max_weight_hyp = -1;
            std::vector<smcl_hyp_t> hyps;
            hyps.resize(pf_->sets[pf_->current_set].cluster_count);
            for (int hyp_count = 0;
                hyp_count < pf_->sets[pf_->current_set].cluster_count; hyp_count++)
            {
                double weight;
                pf_vector_t pose_mean;
                pf_matrix_t pose_cov;
                if (!pf_get_cluster_stats(pf_, hyp_count, &weight, &pose_mean, &pose_cov))
                {
                    ROS_ERROR("Couldn't get stats on cluster %d", hyp_count);
                    break;
                }

                hyps[hyp_count].weight = weight;
                hyps[hyp_count].pf_pose_mean = pose_mean;
                hyps[hyp_count].pf_pose_cov = pose_cov;

                if (hyps[hyp_count].weight > max_weight)
                {
                    max_weight = hyps[hyp_count].weight;
                    max_weight_hyp = hyp_count;
                }
            }

            if (max_weight > 0.0)
            {
                ROS_DEBUG("Max weight pose: %.3f %.3f %.3f",
                          hyps[max_weight_hyp].pf_pose_mean.v[0],
                          hyps[max_weight_hyp].pf_pose_mean.v[1],
                          hyps[max_weight_hyp].pf_pose_mean.v[2]);



                geometry_msgs::PoseWithCovarianceStamped p;
                // Fill in the header
                p.header.frame_id = global_frame_id_;
                p.header.stamp = semantic_map.header.stamp;
                // Copy in the pose
                p.pose.pose.position.x = hyps[max_weight_hyp].pf_pose_mean.v[0];
                p.pose.pose.position.y = hyps[max_weight_hyp].pf_pose_mean.v[1];
                tf::quaternionTFToMsg(tf::createQuaternionFromYaw(hyps[max_weight_hyp].pf_pose_mean.v[2]),
                                      p.pose.pose.orientation);
                // Copy in the covariance, converting from 3-D to 6-D
                pf_sample_set_t* set = pf_->sets + pf_->current_set;
                for (int i = 0; i < 2; i++)
                {
                    for (int j = 0; j < 2; j++)
                    {
                        // Report the overall filter covariance, rather than the
                        // covariance for the highest-weight cluster
                        //p.covariance[6*i+j] = hyps[max_weight_hyp].pf_pose_cov.m[i][j];
                        p.pose.covariance[6 * i + j] = set->cov.m[i][j];
                    }
                }
                // Report the overall filter covariance, rather than the
                // covariance for the highest-weight cluster
                //p.covariance[6*5+5] = hyps[max_weight_hyp].pf_pose_cov.m[2][2];
                p.pose.covariance[6 * 5 + 5] = set->cov.m[2][2];

                tf::Transform transform;
                transform.setOrigin( tf::Vector3(p.pose.pose.position.x, p.pose.pose.position.z, 0.0));
                tf::Quaternion q;
                q.setRPY(0, 0, hyps[max_weight_hyp].pf_pose_mean.v[2]);
                transform.setRotation(q);
                br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), global_frame_id_, base_frame_id_));
            }
            resample_count_ = resample_count_ + 1;
        }
        
    }
    else
    {
        // Pose at last filter update
        pf_odom_pose_ = pose;
        // Filter is now initialized
        pf_init_ = true;
        resample_count_ = 0;
    }
}

void SMCL::initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
    handleInitialPoseMessage(*msg);
}

void SMCL::handleInitialPoseMessage(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    double roll, pitch, yaw;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    ROS_INFO("Setting pose (%.6f): %.3f %.3f %.3f", ros::Time::now().toSec(), msg.pose.pose.position.x, msg.pose.pose.position.y, yaw);
    
    // Re-initialize the filter
    pf_vector_t pf_init_pose_mean = pf_vector_zero();
    pf_init_pose_mean.v[0] = msg.pose.pose.position.x;
    pf_init_pose_mean.v[1] = msg.pose.pose.position.y;
    pf_init_pose_mean.v[2] = yaw;
    pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
    // Copy in the covariance, converting from 6-D to 3-D
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            pf_init_pose_cov.m[i][j] = msg.pose.covariance[6 * i + j];
        }
    }
    pf_init_pose_cov.m[2][2] = msg.pose.covariance[6 * 5 + 5];

    delete initial_pose_hyp_;
    initial_pose_hyp_ = new smcl_hyp_t();
    initial_pose_hyp_->pf_pose_mean = pf_init_pose_mean;
    initial_pose_hyp_->pf_pose_cov = pf_init_pose_cov;
    applyInitialPose();
}

/**
 * If initial_pose_hyp_ and map_ are both non-null, apply the initial
 * pose to the particle filter state.  initial_pose_hyp_ is deleted
 * and set to NULL after it is used.
 */
void SMCL::applyInitialPose()
{
    boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);
    if ( initial_pose_hyp_ != NULL && semantic_map_ != NULL ) {
        pf_init(pf_, initial_pose_hyp_->pf_pose_mean, initial_pose_hyp_->pf_pose_cov);
        pf_init_ = false;

        delete initial_pose_hyp_;
        initial_pose_hyp_ = NULL;
    }
}

bool SMCL::globalLocalizationCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    if ( semantic_map_ == NULL ) {
        return true;
    }
    boost::recursive_mutex::scoped_lock gl(configuration_mutex_);
    ROS_INFO("Initializing with uniform distribution");
    pf_init_model(pf_, (pf_init_model_fn_t)SMCL::uniformPoseGenerator, (void *)semantic_map_);
    ROS_INFO("Global initialisation done!");
    pf_init_ = false;
    return true;
}

// force nomotion updates (smcl updating without requiring motion)
bool SMCL::nomotionUpdateCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    m_force_update = true;
    return true;
}

