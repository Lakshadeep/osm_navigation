#include <sys/types.h> // required by Darwin
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>
#include <vector>
#include <cmath>

#include "smcl/sensors/wall_sides.h"


#include "ros/ros.h"

WallSides::WallSides()
{
}

WallSides::~WallSides()
{
  delete expected_wall_sides;
}


void WallSides::updateMap(semantic_map_t *map)
{
    expected_wall_sides = (wall_side_sensor_t*)malloc(map->no_of_wall_sides * sizeof(wall_side_sensor_t));
    no_of_expected_wall_sides = map->no_of_wall_sides;

    for (int i = 0; i < no_of_expected_wall_sides; i++)
    {
        expected_wall_sides[i].corner1 = map->wall_sides[i].corner1;
        expected_wall_sides[i].corner2 = map->wall_sides[i].corner2;
    }
}

void
WallSides::setModelParams(double z_hit, double z_short, double z_max, double z_rand, double sigma_hit, double lambda_short, double chi_outlier)
{
    this->z_hit = z_hit;
    this->z_short = z_short;
    this->z_max = z_max;
    this->z_rand = z_rand;
    this->sigma_hit = sigma_hit;
    this->lambda_short = lambda_short;
    this->chi_outlier = chi_outlier;
}

bool WallSides::UpdateSensor(pf_t *pf, SensorData *data)
{
    pf_sample_set_t *set;
    set = pf->sets + pf->current_set;
    computeWeights((WallSidesData *)data, set);
    return true;
}


double WallSides::computeWeight(WallSidesData *data, pf_sample_t* sample)
{
    double weight = 1.0;
    for (int i = 0; i < no_of_expected_wall_sides; i++)
    {
        point_t corner1_new, corner2_new;
        double angle, radius;

        corner1_new.x = (expected_wall_sides[i].corner1.x - sample->pose.v[0])*cos(sample->pose.v[2]) + (expected_wall_sides[i].corner1.y - sample->pose.v[1])*sin(sample->pose.v[2]);

        corner1_new.y = -(expected_wall_sides[i].corner1.x - sample->pose.v[0])*sin(sample->pose.v[2]) + (expected_wall_sides[i].corner1.y - sample->pose.v[1])*cos(sample->pose.v[2]);

        corner2_new.x = (expected_wall_sides[i].corner2.x - sample->pose.v[0])*cos(sample->pose.v[2]) + (expected_wall_sides[i].corner2.y - sample->pose.v[1])*sin(sample->pose.v[2]);

        corner2_new.y = -(expected_wall_sides[i].corner2.x - sample->pose.v[0])*sin(sample->pose.v[2]) + (expected_wall_sides[i].corner2.y - sample->pose.v[1])*cos(sample->pose.v[2]);

        getRadiusAngleFromCorners(corner1_new, corner2_new, &angle, &radius);
        // ROS_INFO("Radius %f | Angle %f", radius, angle *180.0/3.14);


        for(int j = 0; j <  data->wall_sides_count; j++)
        {
            double ang_diff = fabs(convert_angle(angle - convert_angle(data->detected_wall_sides[j].angle)));
            double rad_diff = fabs(radius - data->detected_wall_sides[j].radius);

            double pz = 0.0;
            
            double z = rad_diff;
            // Part 1: good, but noisy, hit
            pz += z_hit * exp(-(z * z) / (2 * sigma_hit * sigma_hit));

            // Part 2: short reading from unexpected obstacle (e.g., a person)
            if(z < 0)
              pz += z_short * lambda_short * exp(-lambda_short*data->detected_wall_sides[j].radius);

            // Part 3: Failure to detect obstacle, reported as max-range
            if(data->detected_wall_sides[j].radius == z_max)
              pz += z_max * 1.0;

            // Part 4: Random measurements
            if(data->detected_wall_sides[j].radius < z_max)
              pz += z_rand * 1.0/z_max;

            assert(pz <= 1.0);
            assert(pz >= 0.0);

            weight += pz*pz*pz;
        }
    }
    // ROS_INFO("Weight %f", weight);
    return weight;
}

double WallSides::computeWeights(WallSidesData *data, pf_sample_set_t* set)
{
    double weight;
    pf_sample_t *sample;
    pf_vector_t pose;
    double total_weights;
    for (int i = 0; i < set->sample_count; i++)
    {
        sample = set->samples + i;
        pose = sample->pose; 
        weight = computeWeight(data, sample);
        sample->weight *= weight;
        total_weights = total_weights + weight;
    }
    return total_weights;
}

void WallSides::getRadiusAngleFromCorners(point_t corner1, point_t corner2, double *angle, double *radius)
{
    double slope;
    if (fabs(corner2.x - corner1.x) > 1e-9)
    {
        slope = (corner2.y - corner1.y) / (corner2.x - corner1.x);
        // *angle = pi_to_pi(atan(slope) + M_PI / 2);
        *angle = atan2(corner2.y - corner1.y, corner2.x - corner1.x);
    }
    else
    {
        *angle = 0.0;
    }
    *radius = corner1.x * cos(*angle) + corner1.y * sin(*angle);
    // *radius = fabs(corner1.x * cos(*angle) + corner1.y * sin(*angle));
    if (*radius < 0)
    {
        *radius = -*radius;
        *angle = pi_to_pi(*angle + M_PI);
    }
}

double WallSides::pi_to_pi(double angle)
{
    angle = fmod(angle, 2 * M_PI);
    if (angle >= M_PI)
        angle -= 2 * M_PI;
    return angle;
}

double WallSides::convert_angle(double angle)
{
    if(- 3.14 < angle < 0)
        return (angle + 3.1457);
    return angle;
}

