#include "smcl/sensors/pillars.h"
#include "ros/ros.h"

Pillars::Pillars()
{
}

Pillars::~Pillars()
{
    delete expected_pillars; 
}

// updates expected wall sides using newly updated map
void Pillars::updateMap(semantic_map_t *map)
{
    expected_pillars = (pillar_sensor_t*)malloc(map->no_of_pillars * sizeof(pillar_sensor_t));
    expected_pillars_count = map->no_of_pillars;

    for (int i = 0; i < expected_pillars_count; i++)
    {
        expected_pillars[i].point = map->pillars[i].point;
        // printf("$ExpectedPillar:%d,%f,%f\n",i, expected_pillars[i].point.x, expected_pillars[i].point.y);
    }
}

void Pillars::setSensorParams(double sensor_range_min, double sensor_range_max, double sensor_angular_range_start, double sensor_angular_range_end)
{
    sensor_range_min_ = sensor_range_min;
    sensor_range_max_ = sensor_range_max;
    sensor_angular_range_start_ = sensor_angular_range_start;
    sensor_angular_range_end_ = sensor_angular_range_end;
}

void Pillars::setModelParams(double z_hit, double z_short, double z_max, double z_rand, double sigma_hit, double lambda_short, double chi_outlier)
{
    z_hit_ = z_hit;
    z_short_ = z_short;
    z_max_ = z_max;
    z_rand_ = z_rand;
    sigma_hit_ = sigma_hit;
    lambda_short_ = lambda_short;
    chi_outlier_ = chi_outlier;
}

bool Pillars::UpdateSensor(pf_t *pf, SensorData *data)
{
    pf_sample_set_t *set;
    set = pf->sets + pf->current_set;
    pf_update_sensor(pf, (pf_sensor_model_fn_t) computeWeights, data);
    return true;
}

// registers visible pillars to observed using closest point approach
std::vector<std::pair<pillar_sensor_t, int>> Pillars::registerPillars(std::vector<pillar_sensor_t> visible_pillars, PillarsData *data)
{
    std::vector<std::vector<double>> distance_matrix(data->pillars_count, std::vector<double>(expected_pillars_count));
    for(int j = 0; j < data->pillars_count; j++)
    {
        int i = 0;
        point_t o_pt = polarToCartesian(data->detected_pillars[j].radius, data->detected_pillars[j].angle);
        for (auto v_it = visible_pillars.begin(); v_it != visible_pillars.end(); v_it++)
        {
            point_t v_pt = polarToCartesian(v_it->radius, v_it->angle);            
            distance_matrix[j][i] = calculate_euclidean_distance(v_pt, o_pt);
            i++;
        }
    } 
    std::vector<std::pair<pillar_sensor_t, int>> registered_pillars;
    if(visible_pillars.size() > 0)
    {
        for(int j = 0; j < data->pillars_count; j++)
        { 
            std::vector<double>::iterator result = std::min_element(std::begin(distance_matrix[j]), std::end(distance_matrix[j]));
            ROS_INFO("D1.1");
            int matched_pillar_idx = std::distance(std::begin(distance_matrix[j]), result);
            std::pair<pillar_sensor_t, int> temp(visible_pillars[matched_pillar_idx] , j);
            ROS_INFO("D1.2");
            registered_pillars.push_back(temp);
        }
    }
    return registered_pillars;
}
        
double Pillars::calculate_euclidean_distance(point_t pt1, point_t pt2)
{
    return pow(pow(pt1.x - pt2.x,2) + pow(pt1.y - pt2.y,2),0.5);
}

// used for converting feature to cartesian point for ICP 
point_t Pillars::polarToCartesian(double radius, double angle)
{
    point_t pt;
    pt.x = radius * cos(angle);
    pt.y = radius * sin(angle);
    return pt;
}

// return all visible sides for given sample
std::vector<pillar_sensor_t> Pillars::getVisiblePillars(pf_vector_t sample)
{
    std::vector<pillar_sensor_t> visible_pillars;
    for(int i = 0; i < expected_pillars_count; i++)
    {
        pillar_sensor_t temp;
        temp = expected_pillars[i];
        if (getVisiblePillar(&temp, sample))
        {
            visible_pillars.push_back(temp);
        }
    }
    return visible_pillars;
}

bool Pillars::getVisiblePillar(pillar_sensor_t *pillar, pf_vector_t sample)
{
    pillar->point = globalToLocalTransformation(sample.v[0], sample.v[1], sample.v[2], pillar->point);

    pillar->angle = atan2(pillar->point.y, pillar->point.x);
    pillar->radius = pow(pow(pillar->point.x,2) + pow(pillar->point.y,2), 0.5);
    if (sensor_angular_range_start_ < pillar->angle < sensor_angular_range_end_)
    {        
        if (sensor_range_min_ < pillar->radius < sensor_range_max_)
            return true;
    }
    return false;
}


point_t Pillars::globalToLocalTransformation(double ref_x, double ref_y, double ref_ang, point_t pt)
{
    point_t new_pt;
    new_pt.x = (pt.x - ref_x) * cos(ref_ang) + (pt.y - ref_y) * sin(ref_ang);
    new_pt.y = -(pt.x - ref_x) * sin(ref_ang) + (pt.y - ref_y) * cos(ref_ang);
    return new_pt;
}

double Pillars::computeWeight(PillarsData *data, pf_sample_t *sample)
{
    std::vector<pillar_sensor_t> visible_pillars = getVisiblePillars(sample->pose);
    std::vector<std::pair<pillar_sensor_t, int>> registered_pillars = registerPillars(visible_pillars, data);
    updateSampleFeatures(sample, registered_pillars, data);     

    double pz = 0.0; 
    for (auto reg_it = registered_pillars.begin(); reg_it != registered_pillars.end(); reg_it++)
    {  
        double z = fabs(reg_it->first.radius - data->detected_pillars[reg_it->second].radius);
 
        // Part 1: good, but noisy, hit
        pz = pz + (z_hit_ * exp(-(z * z) / (2 * sigma_hit_ * sigma_hit_)));

        // Part 2: short reading from unexpected obstacle (e.g., a person)
        if(data->detected_pillars[reg_it->second].radius < sensor_range_min_)
          pz = pz + z_short_ * lambda_short_ * exp(-lambda_short_*data->detected_pillars[reg_it->second].radius);

        // Part 3: Failure to detect obstacle, reported as max-range
        if(data->detected_pillars[reg_it->second].radius == z_max_)
          pz = pz + z_max_ * 1.0;

        // Part 4: Random measurements
        if(data->detected_pillars[reg_it->second].radius < z_max_)
          pz = pz + z_rand_ * 1.0/z_max_;
    }
    return pz;
}

bool Pillars::updateSampleFeatures(pf_sample_t *sample, std::vector<std::pair<pillar_sensor_t, int>> registered_pillars, PillarsData *data)
{
    if ( registered_pillars.size() > 0)
    {
        point_t *tmp_expected_features = (point_t*)realloc(sample->expected_features, (sample->no_of_expected_features + registered_pillars.size()) * sizeof(point_t));
        point_t *tmp_registered_features = (point_t*)realloc(sample->registered_features, (sample->no_of_expected_features + registered_pillars.size())* sizeof(point_t));
        if (tmp_expected_features == NULL && tmp_registered_features == NULL)
        {
            return false;
        }
        else
        {
            sample->expected_features = tmp_expected_features;
            sample->registered_features = tmp_registered_features; 
            for(int i = 0; i < registered_pillars.size(); i++)
            {
                sample->expected_features[sample->no_of_expected_features + i] = polarToCartesian(registered_pillars[i].first.radius, registered_pillars[i].first.angle);
                sample->registered_features[sample->no_of_expected_features + i] = polarToCartesian(data->detected_pillars[registered_pillars[i].second].radius, data->detected_pillars[registered_pillars[i].second].angle);
            }

            sample->no_of_expected_features = sample->no_of_expected_features  + registered_pillars.size();
            return true;
        }
    }
    else
      return false;
}

double Pillars::computeWeights(PillarsData *data, pf_sample_set_t* set)
{
    Pillars *self;
    self = (Pillars*) data->sensor;

    // for(int i = 0; i < data->pillars_count; i++)
    // {
    //     printf("$ObservedSide:%f,%f,%f,%f,%f,%f\n", data->detected_pillars[i].corner1.x, data->detected_pillars[i].corner1.y, data->detected_pillars[i].corner2.x, data->detected_pillars[i].corner2.y, data->detected_pillars[i].radius, data->detected_pillars[i].angle);
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
        sample->weight += weight;
        total_weights = total_weights + sample->weight;
    }
    return total_weights;
}

