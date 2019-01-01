#ifndef WALL_SIDES_H
#define WALL_SIDES_H

// C++
#include <sys/types.h> // required by Darwin
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>
#include <vector>
#include <cmath>

// smcl
#include "sensor.h"
#include "../pf/pf_pdf.h"
#include "../map/semantic_map.h"

// structs
typedef struct $
{
    point_t corner1;
    point_t corner2;
    double radius;
    double angle;
}   wall_side_sensor_t;

class WallSidesData : public SensorData
{
public: 
    WallSidesData () {detected_wall_sides=NULL;};
    virtual ~WallSidesData() {delete [] detected_wall_sides;};
    int wall_sides_count;
    double max_range;
    wall_side_sensor_t *detected_wall_sides;
};


class WallSides : public Sensor
{
public: 
    WallSides();
    ~WallSides();
    void setSensorParams(double sensor_range_min, double sensor_range_max, double sensor_angular_range_start, double sensor_angular_range_end);
    void setModelParams(double z_hit, double z_short, double z_max, double z_rand, double sigma_hit, double labda_short, double chi_outlier);
    void updateMap(semantic_map_t *semantic_map);
    virtual bool UpdateSensor(pf_t *pf, SensorData *data);

private:
    // sensor params
    double sensor_range_min_;
    double sensor_range_max_ ;
    double sensor_angular_range_start_;
    double sensor_angular_range_end_;

    // model params
    double z_hit_;
    double z_short_;
    double z_max_;
    double z_rand_;
    double sigma_hit_;
    double lambda_short_;
    double chi_outlier_;

    // cached features for sensor update
    wall_side_sensor_t* expected_wall_sides;
    int expected_wall_sides_count;
    
    // functions for finding visible wall sides for a given sample
    std::vector<wall_side_sensor_t> getVisibleSides(pf_vector_t sample);
    bool getVisibleReorderedSide(wall_side_sensor_t *side, pf_vector_t sample);
    void calculateRadiusAndAngle(wall_side_sensor_t *side);

    // transformation related functions
    double piToPi(double angle);
    double positiveAngle(double angle);
    point_t globalToLocalTransformation(double ref_x, double ref_y, double ref_ang, point_t pt);
    void convertSideToSampleCoordinates(wall_side_sensor_t *side, pf_vector_t sample);

    // particle filter related functions
    double computeWeight(WallSidesData *data, pf_sample_t *sample);
    static double computeWeights(WallSidesData *data, pf_sample_set_t *set);

};

#endif
