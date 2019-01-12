/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey et al.
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef ODOM_H
#define ODOM_H

#include "sensor.h"
#include "../pf/pf_pdf.h"
#include "../map/semantic_map.h"


typedef enum
{
    ODOM_MODEL_DIFF,
    ODOM_MODEL_OMNI,
    ODOM_MODEL_DIFF_CORRECTED,
    ODOM_MODEL_OMNI_CORRECTED
} odom_model_t;

// Odometric sensor data
class OdomData : public SensorData
{
public: 
    pf_vector_t pose;     // Odometric pose
    pf_vector_t delta;    // Change in odometric pose
};


// Odometric sensor model
class Odom : public Sensor
{
    // Default constructor
public: 
    Odom();
    void SetModelDiff(double alpha1, double alpha2, double alpha3, double alpha4);
    void SetModelOmni(double alpha1, double alpha2, double alpha3, double alpha4, double alpha5);
    void SetModel( odom_model_t type, double alpha1, double alpha2, double alpha3, double alpha4, double alpha5 = 0 );
    virtual bool UpdateAction(pf_t *pf, SensorData *data);  // Update the filter based on the action model.  
                                                            // Returns true if the filter has been updated.
    
private: 
    double time;                                     // Current data timestamp
    odom_model_t model_type;                         // Model type
    double alpha1, alpha2, alpha3, alpha4, alpha5;   // Drift parameters
};

#endif
