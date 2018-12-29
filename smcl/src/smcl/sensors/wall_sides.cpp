#include "smcl/sensors/wall_sides.h"
#include "ros/ros.h"

WallSides::WallSides()
{
}

WallSides::~WallSides()
{
}

// updates expected wall sides using newly updated map
void WallSides::updateMap(semantic_map_t *map)
{
    expected_wall_sides.clear();
    for (int i = 0; i < map->no_of_wall_sides; i++)
    {
        wall_side_sensor_t temp;
        temp.corner1 = map->wall_sides[i].corner1;
        temp.corner2 = map->wall_sides[i].corner2;
        expected_wall_sides.push_back(temp);
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
    for(auto wall_it = expected_wall_sides.begin(); wall_it != expected_wall_sides.end(); wall_it++)
    {
        wall_side_sensor_t *temp;
        temp = (wall_side_sensor_t*)malloc(sizeof(wall_side_sensor_t)); 
        temp->corner1 = wall_it->corner1;
        temp->corner2 = wall_it->corner2;
        if (getVisibleReorderedSide(temp, sample))
        {
            visible_sides.push_back(*temp);
        }
    }
    return visible_sides;
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
            side->corner2 = temp;
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
    side->corner1 = globalToLocalTransformation(sample.v[0], sample.v[1], sample.v[2], side->corner2);
}

double WallSides::computeWeight(WallSidesData *data, pf_sample_t *sample)
{
    double weight = 1.0;
    // pf_vector_t *pf_vec = sample->pose;
    std::vector<wall_side_sensor_t> visible_sides = getVisibleSides(sample->pose);

    for (auto visible_it = visible_sides.begin(); visible_it != visible_sides.end(); visible_it++)
    {
        for(int i = 0; i < data->wall_sides_count; i++)
        {
            
            double ang_diff = fabs(visible_it->angle - data->detected_wall_sides[i].angle);
            double rad_diff = fabs(visible_it->radius - data->detected_wall_sides[i].radius);

            if (fabs(visible_it->angle) - 0.3 < data->detected_wall_sides[i].angle < fabs(visible_it->angle) + 0.3 )
            {
                double pz = 0.0;
                double z = rad_diff;

                // Part 1: good, but noisy, hit
                pz += z_hit_ * exp(-(z * z) / (2 * sigma_hit_ * sigma_hit_));

                // Part 2: short reading from unexpected obstacle (e.g., a person)
                if(z < 0)
                  pz += z_short_ * lambda_short_ * exp(-lambda_short_*data->detected_wall_sides[i].radius);

                // Part 3: Failure to detect obstacle, reported as max-range
                if(data->detected_wall_sides[i].radius == z_max_)
                  pz += z_max_ * 1.0;

                // Part 4: Random measurements
                if(data->detected_wall_sides[i].radius < z_max_)
                  pz += z_rand_ * 1.0/z_max_;

                assert(pz <= 1.0);
                assert(pz >= 0.0);

                weight += pz*pz*pz;
            }
        }
    }
    return weight;
}

double WallSides::computeWeights(WallSidesData *data, pf_sample_set_t* set)
{
    WallSides *self;
    self = (WallSides*) data->sensor;

    double weight;
    pf_sample_t *sample;
    pf_vector_t pose;
    double total_weights;
    for (int i = 0; i < set->sample_count; i++)
    {
        sample = set->samples + i;
        pose = sample->pose; 
        weight = self->computeWeight(data, sample);
        sample->weight *= weight;
        total_weights = total_weights + weight;
    }
    return total_weights;
}

