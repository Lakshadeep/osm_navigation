#ifndef WALL_SIDES_H
#define WALL_SIDES_H

#include "sensor.h"
#include "../pf/pf_pdf.h"
#include "../map/semantic_map.h"

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


// wall sides sensor model
class WallSides : public Sensor
{
    // Default constructor
public: 
    WallSides();
    ~WallSides();
    void setModelParams(double z_hit, double z_short, double z_max, double z_rand, double sigma_hit, double labda_short, double chi_outlier);
    void updateMap(semantic_map_t *semantic_map);
    virtual bool UpdateSensor(pf_t *pf, SensorData *data);  // Update the filter based on the sensor model.  
private:
    wall_side_sensor_t *expected_wall_sides;
    int no_of_expected_wall_sides;
    double z_hit;
    double z_short;
    double z_max;
    double z_rand;
    double sigma_hit;
    double lambda_short;
    double chi_outlier; 
    semantic_map_t semantic_map;
    double computeWeight(WallSidesData *data, pf_sample_t* sample);
    double computeWeights(WallSidesData *data, pf_sample_set_t* set);
    void getRadiusAngleFromCorners(point_t corner1, point_t corner2, double *angle, double *radius);
    double pi_to_pi(double angle);
    double convert_angle(double angle);
};

#endif
