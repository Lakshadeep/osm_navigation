#ifndef PILLARS_H
#define PILLARS_H

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
typedef struct
{
    point_t point;
    double radius;
    double angle;
}   pillar_sensor_t;

class PillarsData : public SensorData
{
public: 
    PillarsData () {detected_pillars=NULL;};
    virtual ~PillarsData() {delete [] detected_pillars;};
    int pillars_count;
    double max_range;
    pillar_sensor_t *detected_pillars;
};


class Pillars : public Sensor
{
public: 
    Pillars();
    ~Pillars();
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
    pillar_sensor_t* expected_pillars;
    int expected_pillars_count;
    
    // functions for finding visible pillars for a given sample
    std::vector<pillar_sensor_t> getVisiblePillars(pf_vector_t sample);
    bool getVisiblePillar(pillar_sensor_t *pillar, pf_vector_t sample);

    // transformation related functions
    point_t globalToLocalTransformation(double ref_x, double ref_y, double ref_ang, point_t pt);


    // particle filter related functions
    double computeWeight(PillarsData *data, pf_sample_t *sample);
    static double computeWeights(PillarsData *data, pf_sample_set_t *set);

};

#endif
