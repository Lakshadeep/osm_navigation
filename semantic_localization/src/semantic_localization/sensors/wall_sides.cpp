#include "semantic_localization/sensors/wall_sides.h"
#include "ros/ros.h"

WallSides::WallSides(int feature_num)
{
    feature_type_no_ = feature_num;
}

WallSides::~WallSides()
{
    delete expected_wall_sides; 
}

// updates expected wall sides using newly updated map
void WallSides::updateMap(semantic_map_t *map)
{
    expected_wall_sides = (wall_side_sensor_t*)malloc(map->no_of_wall_sides * sizeof(wall_side_sensor_t));
    expected_wall_sides_count = map->no_of_wall_sides;

    for (int i = 0; i < expected_wall_sides_count; i++)
    {
        expected_wall_sides[i].corner1 = map->wall_sides[i].corner1;
        expected_wall_sides[i].corner2 = map->wall_sides[i].corner2;
        // printf("$ExpectedSide:%d,%f,%f,%f,%f\n",i, expected_wall_sides[i].corner1.x, expected_wall_sides[i].corner1.y, expected_wall_sides[i].corner2.x, expected_wall_sides[i].corner2.y);
    }
}

void WallSides::setSensorParams(double sensor_range_min, double sensor_range_max, double sensor_angular_range_start, double sensor_angular_range_end)
{
    sensor_range_min_ = sensor_range_min;
    sensor_range_max_ = sensor_range_max;
    sensor_angular_range_start_ = sensor_angular_range_start;
    sensor_angular_range_end_ = sensor_angular_range_end;
}

void WallSides::setModelParams(double z_hit, double z_short, double z_max, double z_rand, double sigma_hit, double lambda_short, double chi_outlier)
{
    z_hit_ = z_hit;
    z_short_ = z_short;
    z_max_ = z_max;
    z_rand_ = z_rand;
    sigma_hit_ = sigma_hit;
    lambda_short_ = lambda_short;
    chi_outlier_ = chi_outlier;
}

bool WallSides::UpdateSensor(pf_t *pf, SensorData *data)
{
    pf_sample_set_t *set;
    set = pf->sets + pf->current_set;
    pf_update_sensor(pf, (pf_sensor_model_fn_t) computeWeights, data);
    return true;
}

// return all visible sides for given sample
std::vector<wall_side_sensor_t> WallSides::getVisibleSides(pf_vector_t sample)
{
    std::vector<wall_side_sensor_t> visible_sides;
    for(int i = 0; i < expected_wall_sides_count; i++)
    {
        wall_side_sensor_t temp;
        temp = expected_wall_sides[i];                // TODO : check this!
        if (getVisibleReorderedSide(&temp, sample))
        {
            visible_sides.push_back(temp);
        }
    }
    return visible_sides;
}

// registers visible sides to observed using closest point approach
std::vector<std::pair<wall_side_sensor_t, int>> WallSides::registerWallSides(std::vector<wall_side_sensor_t> visible_sides, WallSidesData *data)
{
    std::vector<std::vector<double>> distance_matrix(data->wall_sides_count, std::vector<double>(expected_wall_sides_count));
    
    for(int j = 0; j < data->wall_sides_count; j++)
    {
        int i = 0;
        point_t o_pt = polarToCartesian(data->detected_wall_sides[j].radius, data->detected_wall_sides[j].angle);
        for (auto v_it = visible_sides.begin(); v_it != visible_sides.end(); v_it++)
        {
            point_t v_pt = polarToCartesian(v_it->radius, v_it->angle);            
            distance_matrix[j][i] = calculate_euclidean_distance(v_pt, o_pt);
            i++;
        }
    } 

    std::vector<std::pair<wall_side_sensor_t, int>> registered_wall_sides;
    for(int j = 0; j < data->wall_sides_count; j++)
    { 
        std::vector<double>::iterator result = std::min_element(std::begin(distance_matrix[j]), std::end(distance_matrix[j]));
        int matched_side_idx = std::distance(std::begin(distance_matrix[j]), result);
        if (distance_matrix[j][matched_side_idx] < 2.0)
        {
            std::pair<wall_side_sensor_t, int> temp(visible_sides[matched_side_idx] , j);
            registered_wall_sides.push_back(temp);
        }
    }

    return registered_wall_sides;
}
        
double WallSides::calculate_euclidean_distance(point_t pt1, point_t pt2)
{
    return pow(pow(pt1.x - pt2.x,2) + pow(pt1.y - pt2.y,2),0.5);
}

// used for converting feature to cartesian point for ICP 
point_t WallSides::polarToCartesian(double radius, double angle)
{
    point_t pt;
    pt.x = radius * cos(angle);
    pt.y = radius * sin(angle);
    return pt;
}

// finds visible sides. Input sides should be in robot coordinate system
// pass by reference
// return true indicates side is visible
bool WallSides::getVisibleReorderedSide(wall_side_sensor_t *side, pf_vector_t sample)
{
    convertSideToSampleCoordinates(side, sample);

    double angle_corner1 = positiveAngle(atan2(side->corner1.y, side->corner1.x));
    double angle_corner2 = positiveAngle(atan2(side->corner2.y, side->corner2.x));

    if ((sensor_angular_range_start_ < angle_corner1) || (sensor_angular_range_start_ < angle_corner2) || (0 < angle_corner1 < sensor_angular_range_end_) || (0 < angle_corner2 < sensor_angular_range_end_))
    {
        if (angle_corner1 > angle_corner2)
        {
            point_t temp = side->corner2;
            side->corner2 = side->corner1;
            side->corner1 = temp;
        }
        calculateRadiusAndAngle(side);
        
        if (sensor_range_min_ < side->radius < sensor_range_max_)
            return true;
    }
    return false;
}

// return side angle and radius
// pass by reference
void WallSides::calculateRadiusAndAngle(wall_side_sensor_t *side)
{
    double slope;
    if (fabs(side->corner2.x - side->corner1.x) > 1e-9)
    {
        slope = (side->corner2.y - side->corner1.y) / (side->corner2.x - side->corner1.x);
        side->angle = piToPi(atan(slope) + M_PI / 2);
    }
    else
    {
        side->angle = 0.0;
    }
    side->radius = side->corner1.x * cos(side->angle) + side->corner1.y * sin(side->angle);
    if (side->radius < 0)
    {
        side->radius = -side->radius;
        side->angle = piToPi(side->angle + M_PI);
    }
}

double WallSides::piToPi(double angle)
{
    angle = fmod(angle, 2 * M_PI);
    if (angle >= M_PI)
        angle -= 2 * M_PI;
    return angle;
}

double WallSides::positiveAngle(double angle)
{
    if(angle < 0)
        return fabs(fmod(angle, 2 * M_PI) + 2 * M_PI);
    else
        return fabs(fmod(angle, 2 * M_PI));
}

point_t WallSides::globalToLocalTransformation(double ref_x, double ref_y, double ref_ang, point_t pt)
{
    point_t new_pt;
    new_pt.x = (pt.x - ref_x) * cos(ref_ang) + (pt.y - ref_y) * sin(ref_ang);
    new_pt.y = -(pt.x - ref_x) * sin(ref_ang) + (pt.y - ref_y) * cos(ref_ang);
    return new_pt;
}

void WallSides::convertSideToSampleCoordinates(wall_side_sensor_t *side, pf_vector_t sample)
{
    side->corner1 = globalToLocalTransformation(sample.v[0], sample.v[1], sample.v[2], side->corner1);
    side->corner2 = globalToLocalTransformation(sample.v[0], sample.v[1], sample.v[2], side->corner2);
}

bool WallSides::compare_pair(std::pair<double, int> p1, std::pair<double, int> p2)
{
    return p1.second > p2.second;
}

double WallSides::computeWeight(WallSidesData *data, pf_sample_t *sample)
{
    std::vector<wall_side_sensor_t> visible_sides = getVisibleSides(sample->pose);
    std::vector<std::pair<wall_side_sensor_t, int>> registered_sides = registerWallSides(visible_sides, data);
    updateSampleFeatures(sample, registered_sides, data);     

    double w = 0.0; 
    std::vector<std::pair<double,int>> detection_weights;
    for (auto reg_it = registered_sides.begin(); reg_it != registered_sides.end(); reg_it++)
    {  
        double pz = 1.0;
        double z_radial = fabs(reg_it->first.radius - data->detected_wall_sides[reg_it->second].radius);
        double z_bearing = fabs(reg_it->first.angle - data->detected_wall_sides[reg_it->second].angle);

        // Part 1: confusion matrix (prob. of wall side given wall side)
        pz *= 0.8;

        // Part 2: observation likelihood based on radial difference
        double ol1 = (z_hit_ * exp(-(z_radial * z_radial) / (2 * sigma_hit_ * sigma_hit_)));

        // Part 3: observation likelihood based on bearing difference
        double ol2 = (z_hit_ * exp(-(z_bearing * z_bearing) / (2 * sigma_hit_ * sigma_hit_)));

        pz *= (ol1 + ol2);

        // Part 4: false detection
        if( data->detected_wall_sides[reg_it->second].radius < sensor_range_min_)
            pz = pz + z_short_ * lambda_short_ * exp(-lambda_short_*data->detected_wall_sides[reg_it->second].radius);

        // Part 5: Random measurements
        if( data->detected_wall_sides[reg_it->second].radius < z_max_)
            pz = pz + z_rand_ * 1.0/z_max_;

        std::pair<double, int> temp(pz, reg_it->second);
        detection_weights.push_back(temp);
    }

    // to average out weight of same feature detected multiple times
    std::sort(detection_weights.begin(), detection_weights.end(), compare_pair);
    double last_weight = 0.0;
    int last_vis_side = -1;
    int count = 0;
    for (auto dw_it = detection_weights.begin(); dw_it != detection_weights.end(); dw_it++)
    {
        if( dw_it->second == last_vis_side)
        {
            if(count == 1)
                w = w - last_weight;  
            last_weight = last_weight + dw_it->first;
            count++;
        }
        else
        { 
            if( count > 1 )
                w = w + (last_weight/count);
            w = w + dw_it->first;
            last_weight = dw_it->first;
            count = 1;
            last_vis_side = dw_it->second;
        }
    }
    if(count > 1)
        w = w + last_weight;


    for(auto visible_sides_it = visible_sides.begin(); visible_sides_it != visible_sides.end(); visible_sides_it++)
    {
        bool is_detected = false;
        int visible_side_no = 0;
        double pz = 1.0;
        for (auto reg_it = registered_sides.begin(); reg_it != registered_sides.end(); reg_it++)
        { 
            if(reg_it->second == visible_side_no)
            {
                is_detected = true;
                break;
            }
            visible_side_no++;
        }
        if (!is_detected && registered_sides.size() > 0)
        {
            pz *= 0.2;
            w -= pz;
        }
    }
    return w;
}

bool WallSides::updateSampleFeatures(pf_sample_t *sample, std::vector<std::pair<wall_side_sensor_t, int>> registered_sides, WallSidesData *data)
{
    if ( registered_sides.size() > 0)
    {
        point_t *tmp_expected_features = (point_t*)realloc(sample->expected_features, (sample->no_of_expected_features + registered_sides.size()) * sizeof(point_t));
        point_t *tmp_registered_features = (point_t*)realloc(sample->registered_features, (sample->no_of_expected_features + registered_sides.size())* sizeof(point_t));
        if (tmp_expected_features == NULL && tmp_registered_features == NULL)
        {
            return false;
        }
        else
        {
            sample->expected_features = tmp_expected_features;
            sample->registered_features = tmp_registered_features; 
            for(int i = 0; i < registered_sides.size(); i++)
            {
                sample->expected_features[sample->no_of_expected_features + i] = polarToCartesian(registered_sides[i].first.radius, registered_sides[i].first.angle);
                sample->registered_features[sample->no_of_expected_features + i] = polarToCartesian(data->detected_wall_sides[registered_sides[i].second].radius, data->detected_wall_sides[registered_sides[i].second].angle);
            }

            sample->no_of_expected_features = sample->no_of_expected_features  + registered_sides.size();
            return true;
        }
    }
    else
        return false;
}

double WallSides::computeWeights(WallSidesData *data, pf_sample_set_t* set)
{
    WallSides *self;
    self = (WallSides*) data->sensor;

    // for(int i = 0; i < data->wall_sides_count; i++)
    // {
    //     printf("$ObservedSide:%f,%f,%f,%f,%f,%f\n", data->detected_wall_sides[i].corner1.x, data->detected_wall_sides[i].corner1.y, data->detected_wall_sides[i].corner2.x, data->detected_wall_sides[i].corner2.y, data->detected_wall_sides[i].radius, data->detected_wall_sides[i].angle);
    // }

    double weight;
    pf_sample_t *sample;
    pf_vector_t pose;
    double total_weights;
    for (int i = 0; i < set->sample_count; i++)
    {
        sample = set->samples + i;
        pose = sample->pose; 
        weight = self->computeWeight(data, sample);
        sample->feature_types_count = self->feature_type_no_;
        double* realloc_status = (double*)realloc(sample->weights, sample->feature_types_count * sizeof(double));
        if(realloc_status !=NULL)
        {
            sample->weights = realloc_status;
            sample->weights[sample->feature_types_count - 1] = weight;
        }
        else
            ROS_ERROR("Reallocation failed");
    }
    return total_weights;
}

